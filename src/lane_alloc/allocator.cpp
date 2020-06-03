/**
 * \file allocator.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * This file is part of SILICON.
 *
 * SILICON is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * SILICON is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * SILICON.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/lane_alloc/allocator.hpp"

#include <algorithm>

#include "silicon/controller/perception/ct_skel_info.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, lane_alloc);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
allocator::allocator(const config::lane_alloc_config* config, rmath::rng* rng)
    : ER_CLIENT_INIT("silicon.lane_alloc.allocator"),
      mc_config(*config),
      m_rng(rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::vector<lane_geometry> allocator::lane_locs_calc(
    const scperception::ct_skel_info* target) const {
  std::vector<lane_geometry> ret;

  /*
   * For all orientations, the actual ingress points are in the cell that is one
   * unit OUTSIDE of the actual target (a virtual cell) in some direction, so we
   * compute that, and let the \ref lane_geometry class compute everything else.
   */
  auto bbd = target->bbd(true);
  if (rmath::radians::kZERO == target->orientation()) {
    for (size_t j = target->vshell_sized(); j < bbd.y() - target->vshell_sized(); j += 2) {
      rmath::vector3z ingress_nearest(bbd.x() - 1, j, 0);
      rmath::vector3z egress_nearest(bbd.x() - 1, j + 1, 0);

      auto geometry = lane_geometry(target,
                                    ingress_nearest,
                                    egress_nearest);
      ret.push_back(std::move(geometry));
    } /* for(j..) */
  } else if (rmath::radians::kPI_OVER_TWO == target->orientation()) {
    for (size_t i = target->vshell_sized(); i < bbd.x() - target->vshell_sized(); i += 2) {
      rmath::vector3z ingress_nearest(i + 1, bbd.y() - 1, 0);
      rmath::vector3z egress_nearest(i, bbd.y() - 1, 0);

      auto geometry = lane_geometry(target,
                                    ingress_nearest,
                                    egress_nearest);
      ret.push_back(std::move(geometry));
    } /* for(i..) */
  } else if (rmath::radians::kPI == target->orientation()) {
    for (size_t j = target->vshell_sized(); j < bbd.y() - target->vshell_sized(); j += 2) {
      rmath::vector3z ingress_nearest(0, j + 1, 0);
      rmath::vector3z egress_nearest(0, j, 0);

      auto geometry = lane_geometry(target,
                                    ingress_nearest,
                                    egress_nearest);
      ret.push_back(std::move(geometry));
    } /* for(j..) */
  } else if (rmath::radians::kTHREE_PI_OVER_TWO == target->orientation()) {
    for (size_t i = target->vshell_sized(); i < bbd.x() - target->vshell_sized(); i += 2) {
      rmath::vector3z ingress_nearest(i, 0, 0);
      rmath::vector3z egress_nearest(i + 1, 0, 0);

      auto geometry = lane_geometry(target,
                                    ingress_nearest,
                                    egress_nearest);
      ret.push_back(std::move(geometry));
    } /* for(i..) */
  } else {
    ER_FATAL_SENTINEL("Bad lane orientation '%s'",
                      rcppsw::to_string(target->orientation()).c_str());
  }
  return ret;
} /* lane_locs_calc() */

std::unique_ptr<repr::construction_lane> allocator::operator()(
    const rmath::vector3d& robot_loc,
    const scperception::ct_skel_info* target) {
  auto locs = lane_locs_calc(target);

  /*
   * If we have never allocated a lane from this structure before, then
   * initialize a new allocation history for the structure.
   */
  auto hist_it = m_history.find(target->id());
  if (m_history.end() == hist_it) {
    allocation_history h(locs.size(), m_rng->uniform(0UL, locs.size() - 1));
    m_history.insert({target->id(), h});
  }
  auto& hist = m_history.find(target->id())->second;

  size_t id = 0;
  if (kPolicyRandom == mc_config.policy) {
    id = m_rng->uniform(0UL, locs.size() - 1);
  } else if (kPolicyLRU == mc_config.policy) {
    id = (hist.prev_lane + 1) % locs.size();
    hist.prev_lane = id;
  } else if (kPolicyClosest == mc_config.policy) {
    auto pred = [&](const auto& loc1, const auto& loc2) {
      return (robot_loc - loc1.ingress_start()).length() <
             (robot_loc - loc2.ingress_start()).length();
    };
    auto it = std::min_element(locs.begin(), locs.end(), pred);
    id = std::distance(locs.begin(), it);
  } else {
    ER_FATAL_SENTINEL("Bad lane allocation policy '%s'",
                      mc_config.policy.c_str());
  }
  ++hist.alloc_counts[id];

  ER_INFO("Allocated lane%zu: orientation=%s, ingress=%s, egress=%s",
          id,
          rcppsw::to_string(target->orientation()).c_str(),
          rcppsw::to_string(locs[id].ingress_start()).c_str(),
          rcppsw::to_string(locs[id].egress_start()).c_str());
  return std::make_unique<repr::construction_lane>(
      id,
      target->orientation(),
      locs[id].ingress_start(),
      locs[id].egress_start(),
      locs[id].ingress_cell(),
      locs[id].egress_cell());
} /* operator()() */

/*******************************************************************************
 * Metrics
 ******************************************************************************/
size_t allocator::alloc_count(const rtypes::type_uuid& target, size_t id) const {
  auto it = m_history.find(target);
  if (m_history.end() != it) {
    return it->second.alloc_counts[id];
  }
  return 0;
} /* alloc_count() */

void allocator::reset_metrics(void) {
  for (auto& pair : m_history) {
    std::fill(pair.second.alloc_counts.begin(),
              pair.second.alloc_counts.end(),
              0);
  } /* for(&pair..) */
} /* reset_metrics() */

NS_END(lane_alloc, silicon);
