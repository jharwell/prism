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
std::vector<allocator::lane_locs> allocator::lane_locs_calc(
    const sstructure::structure3D* structure) const {
  std::vector<lane_locs> ret;
  if (rmath::radians::kZERO == structure->orientation()) {
    for (size_t j = 0; j < structure->ydsize(); j+=2) {
      auto& ingress_cell = structure->access(structure->origind() +
                                             rmath::vector3z(0, j, 0));
      auto& egress_cell = structure->access(structure->origind() +
                                            rmath::vector3z(0, j + 1, 0));
      ret.push_back({structure->cell_loc_abs(ingress_cell),
              structure->cell_loc_abs(egress_cell)});
    } /* for(j..) */
  } else if (rmath::radians::kPI_OVER_TWO == structure->orientation()) {
    for (size_t i = 0; i < structure->ydsize(); i+=2) {
      auto& ingress_cell = structure->access(structure->origind() + rmath::vector3z(i, 0, 0));
      auto& egress_cell = structure->access(structure->origind() + rmath::vector3z(i + 1, 0, 0));
      ret.push_back({structure->cell_loc_abs(ingress_cell),
              structure->cell_loc_abs(egress_cell)});
    } /* for(i..) */
  }
  return ret;
} /* lane_locs_calc() */

repr::construction_lane allocator::operator()(
    const rmath::vector3d& robot_loc,
    const sstructure::structure3D* structure) {
  auto locs = lane_locs_calc(structure);

  /*
   * If we have never allocated a lane from this structure before, then
   * initialize a new allocation history for the structure
   */
  auto hist_it = m_history.find(structure->id());
  if (m_history.end() == hist_it) {
    allocation_history h(locs.size(),
                         m_rng->uniform(0, locs.size() - 1));
    m_history.insert({structure->id(), h});
  }
  auto& hist = m_history.find(structure->id())->second;

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

  return {id, structure->orientation(), locs[id].ingress, locs[id].egress};
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
