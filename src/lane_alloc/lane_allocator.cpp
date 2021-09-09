/**
 * \file lane_allocator.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * This file is part of PRISM.
 *
 * PRISM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * PRISM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * PRISM.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/lane_alloc/lane_allocator.hpp"

#include <algorithm>

#include "prism/controller/perception/ct_skel_info.hpp"
#include "prism/lane_alloc/random_allocator.hpp"
#include "prism/lane_alloc/lru_allocator.hpp"
#include "prism/lane_alloc/interference_allocator.hpp"
#include "prism/lane_alloc/closest_allocator.hpp"
#include "prism/gmt/utils.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, lane_alloc);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
lane_allocator::lane_allocator(const config::lane_alloc_config* config, rmath::rng* rng)
    : ER_CLIENT_INIT("prism.lane_alloc.lane_allocator"),
      mc_config(*config),
      m_rng(rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::vector<lane_geometry>
lane_allocator::lane_locs_calc(const pcperception::ct_skel_info* target) const {
  std::vector<lane_geometry> ret;

    ER_ASSERT(pgmt::orientation_valid(target->orientation()),
            "Bad orientation: '%s'",
            rcppsw::to_string(target->orientation()).c_str());

  /*
   * For all orientations, the actual ingress points are in the cell that is one
   * unit OUTSIDE of the actual target (a virtual cell) in some direction, so we
   * compute that, and let the \ref lane_geometry class compute everything else.
   */
  auto bbd = target->bbd(true);
  if (rmath::radians::kZERO == target->orientation()) {
    for (size_t j = target->vshell_sized(); j < bbd.y() - target->vshell_sized();
         j += 2) {
      auto ingress_virt = target->as_vcoord({ 0, j + 1, 0 });
      auto egress_virt = target->as_vcoord({ 0, j, 0 });
      ret.emplace_back(target, ingress_virt, egress_virt);
    } /* for(j..) */
  } else if (rmath::radians::kPI_OVER_TWO == target->orientation()) {
    for (size_t i = target->vshell_sized(); i < bbd.x() - target->vshell_sized();
         i += 2) {
      auto ingress_virt = target->as_vcoord({ i, 0, 0 });
      auto egress_virt = target->as_vcoord({ i + 1, 0, 0 });

      ret.emplace_back(target, ingress_virt, egress_virt);
    } /* for(i..) */
  } else if (rmath::radians::kPI == target->orientation()) {
    for (size_t j = target->vshell_sized(); j < bbd.y() - target->vshell_sized();
         j += 2) {
      auto ingress_virt = target->as_vcoord({ bbd.x() - 1, j, 0 });
      auto egress_virt = target->as_vcoord({ bbd.x() - 1, j + 1, 0 });

      ret.emplace_back(target, ingress_virt, egress_virt);
    } /* for(j..) */
  } else if (rmath::radians::kTHREE_PI_OVER_TWO == target->orientation()) {
    for (size_t i = target->vshell_sized(); i < bbd.x() - target->vshell_sized();
         i += 2) {
      auto ingress_virt = target->as_vcoord({ i + 1, bbd.y() - 1, 0 });
      auto egress_virt = target->as_vcoord({ i, bbd.y() - 1, 0 });

      ret.emplace_back(target, ingress_virt, egress_virt);
    } /* for(i..) */
  }
  return ret;
} /* lane_locs_calc() */

std::unique_ptr<repr::construction_lane>
lane_allocator::operator()(const rmath::vector3d& robot_pos,
                      const pcperception::ct_skel_info* target) {
  auto locs = lane_locs_calc(target);

  /*
   * If we have never allocated a lane from this structure before, then
   * initialize a new allocation history for the gmt, and set the last
   * allocated lane randomly.
   */
  auto hist_it = m_history.find(target->id());
  if (m_history.end() == hist_it) {
    history h(locs.size(), m_rng);
    m_history.insert({ target->id(), h });
  }
  auto& hist = m_history.find(target->id())->second;

  size_t id = 0;
  if (kPolicyRandom == mc_config.policy) {
    id = random_allocator(m_rng)(locs);
  } else if (kPolicyLRU == mc_config.policy) {
    id = lru_allocator(m_rng)(locs, &hist);
  } else if (kPolicyClosest == mc_config.policy) {
    id = closest_allocator(m_rng)(locs, robot_pos);
  } else if (kPolicyMinInterference == mc_config.policy) {
    id = interference_allocator(m_rng)(locs, &hist);
  } else {
    ER_FATAL_SENTINEL("Bad lane allocation policy '%s'",
                      mc_config.policy.c_str());
  }
  hist.alloc_mark(id);

  ER_INFO("Allocated lane%zu: orientation=%s, ingress=%s, egress=%s",
          id,
          rcppsw::to_string(target->orientation()).c_str(),
          rcppsw::to_string(locs[id].ingress_pt()).c_str(),
          rcppsw::to_string(locs[id].egress_pt()).c_str());
  return std::make_unique<repr::construction_lane>(
      id, target->orientation(), locs[id], &hist);
} /* operator()() */

/*******************************************************************************
 * Metrics
 ******************************************************************************/
size_t lane_allocator::alloc_count(const rtypes::type_uuid& target, size_t id) const {
  auto it = m_history.find(target);
  if (m_history.end() != it) {
    return it->second.alloc_count(id);
  }
  return 0;
} /* alloc_count() */

void lane_allocator::reset_metrics(void) {
  for (auto& pair : m_history) {
    pair.second.reset_int_allocs();
  } /* for(&pair..) */
} /* reset_metrics() */

NS_END(lane_alloc, prism);
