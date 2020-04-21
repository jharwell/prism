/**
 * \file construction_lane_allocator.cpp
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
#include "silicon/controller/construction_lane_allocator.hpp"

#include <algorithm>

#include "silicon/structure/structure3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, controller);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
construction_lane_allocator::construction_lane_allocator(
    const config::lane_alloc_config* config,
    const sstructure::structure3D* structure,
    rmath::rng* rng)
    : ER_CLIENT_INIT("silicon.controller.construction_lane_allocator"),
      mc_config(*config),
      mc_lane_locs(lane_locs_calc(structure)),
      m_prev_lane(rng->uniform(0, mc_lane_locs.size() - 1)),
      m_rng(rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::vector<rmath::vector3d> construction_lane_allocator::lane_locs_calc(
    const sstructure::structure3D* structure) const {
  std::vector<rmath::vector3d> ret;
  if (rmath::radians::kZERO == structure->orientation()) {
    for (size_t j = 0; j < structure->ysize(); j+=2) {
      auto& cell = structure->access(structure->origind() + rmath::vector3z(0, j, 0));
          ret.push_back(structure->cell_loc_abs(cell));
    } /* for(j..) */
  } else if (rmath::radians::kPI_OVER_TWO == structure->orientation()) {
    for (size_t i = 0; i < structure->ysize(); i+=2) {
      auto& cell = structure->access(structure->origind() + rmath::vector3z(i, 0, 0));
      ret.push_back(structure->cell_loc_abs(cell));
    } /* for(i..) */
  }
  return ret;
} /* lane_locs_calc() */

size_t construction_lane_allocator::operator()(
    const rmath::vector3d& robot_loc) {
  size_t ret = 0;
  if (kPolicyRandom == mc_config.policy) {
    ret = m_rng->uniform(0, mc_lane_locs.size() - 1);
  } else if (kPolicyLRU == mc_config.policy) {
    ret = (m_prev_lane + 1) % mc_lane_locs.size();
    m_prev_lane = ret;
  } else if (kPolicyClosest == mc_config.policy) {
    auto pred = [&](const auto& loc1, const auto& loc2) {
      return (robot_loc - loc1).length() < (robot_loc - loc2).length();
    };
    auto it = std::min_element(mc_lane_locs.begin(),
                               mc_lane_locs.end(),
                               pred);
    ret = std::distance(mc_lane_locs.begin(), it);
  } else {
    ER_FATAL_SENTINEL("Bad lane allocation policy '%s'",
                      mc_config.policy.c_str());
  }
  ++m_alloc_counts[ret];
  return ret;
} /* operator()() */

NS_END(controller, silicon);
