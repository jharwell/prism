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

#include "silicon/structure/structure3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, lane_alloc);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
allocator::allocator(
    const config::lane_alloc_config* config,
    rmath::rng* rng)
    : ER_CLIENT_INIT("silicon.lane_alloc.allocator"),
      mc_config(*config),
      m_rng(rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::vector<allocator::lane_geometry> allocator::lane_locs_calc(
     const scperception::ct_skel_info* target) const {
  std::vector<lane_geometry> ret;

  /*
   * Need to account for block width so we calculate the CENTER of the
   * ingress/egress lanes.
   */
  rmath::vector3d correction(target->block_unit_dim() / 2.0,
                             target->block_unit_dim() / 2.0,
                             0.0);

  if (rmath::radians::kZERO == target->orientation()) {
    for (size_t j = 0; j < target->bbd().y(); j+=2) {
      auto ingress = target->cell_loc_abs({target->bbd().x() - 1, j, 0}) +
                     rmath::vector3d::X* target->block_unit_dim();
      auto egress = target->cell_loc_abs({target->bbd().x() - 1, j + 1, 0}) +
                    rmath::vector3d::X* target->block_unit_dim();
      rmath::vector3d center(target->originr().x() +
                             (ingress.x() - target->originr().x()) / 2.0,
                             ingress.y(),
                             0.0);

      ret.push_back({ingress + correction,
              egress + correction,
              center + correction});
    } /* for(j..) */
  } else if (rmath::radians::kPI_OVER_TWO == target->orientation()) {
    for (size_t i = 0; i < target->bbd().x(); i+=2) {
      auto ingress = target->cell_loc_abs({i, target->bbd().y() - 1, 0}) -
                     rmath::vector3d::Y* target->block_unit_dim();
      auto egress = target->cell_loc_abs({i + 1, target->bbd().y() - 1, 0}) -
                    rmath::vector3d::Y* target->block_unit_dim();
      rmath::vector3d center(ingress.x(),
                             target->originr().y() -
                             (ingress.y() - target->originr().y()) / 2.0,
                             0.0);
      ret.push_back({ingress + correction,
              egress + correction,
              center + correction});
    } /* for(i..) */
  }
  return ret;
} /* lane_locs_calc() */

repr::construction_lane allocator::operator()(
    const rmath::vector3d& robot_loc,
    const scperception::ct_skel_info* target) {
  auto locs = lane_locs_calc(target);

  /*
   * If we have never allocated a lane from this structure before, then
   * initialize a new allocation history for the structure.
   */
  auto hist_it = m_history.find(target->id());
  if (m_history.end() == hist_it) {
    allocation_history h(locs.size(),
                         m_rng->uniform(0, locs.size() - 1));
    m_history.insert({target->id(), h});
  }
  auto& hist = m_history.find(target->id())->second;

  size_t id = 0;
  if (kPolicyRandom == mc_config.policy) {
    id = m_rng->uniform(0, locs.size() - 1);
  } else if (kPolicyLRU == mc_config.policy) {
    id = (hist.prev_lane + 1) % locs.size();
    hist.prev_lane = id;
  } else if (kPolicyClosest == mc_config.policy) {
    auto pred = [&](const auto& loc1, const auto& loc2) {
      return (robot_loc - loc1.ingress).length() < (robot_loc - loc2.ingress).length();
    };
    auto it = std::min_element(locs.begin(),
                               locs.end(),
                               pred);
    id = std::distance(locs.begin(), it);
  } else {
    ER_FATAL_SENTINEL("Bad lane allocation policy '%s'",
                      mc_config.policy.c_str());
  }
  ++hist.alloc_counts[id];

  ER_INFO("Allocated lane%zu: orientation=%s, ingress=%s, egress=%s",
          id,
          rcppsw::to_string(target->orientation()).c_str(),
          rcppsw::to_string(locs[id].ingress).c_str(),
          rcppsw::to_string(locs[id].egress).c_str());
  return repr::construction_lane(id,
                                 target->orientation(),
                                 locs[id].ingress,
                                 locs[id].egress);
} /* operator()() */

/*******************************************************************************
 * Metrics
 ******************************************************************************/
size_t allocator::alloc_count(const rtypes::type_uuid& target,
                              size_t id) const {
  auto it = m_history.find(target);
  if (m_history.end() != it) {
    return it->second.alloc_counts[id];
  }
  return 0;
} /* alloc_count() */

NS_END(lane_alloc, silicon);
