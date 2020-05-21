/**
 * \file placement_path_calculator.cpp
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
#include "silicon/fsm/placement_path_calculator.hpp"

#include "rcppsw/math/radians.hpp"

#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

#include "silicon/repr/construction_lane.hpp"
#include "silicon/controller/perception/builder_perception_subsystem.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
placement_path_calculator::placement_path_calculator(
    const csubsystem::sensing_subsystemQ3D* sensing,
    const scperception::builder_perception_subsystem* perception)
    : ER_CLIENT_INIT("silicon.fsm.placement_path_calculator"),
      mc_sensing(sensing),
      mc_perception(perception) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::vector<rmath::vector2d> placement_path_calculator::operator()(
    const srepr::construction_lane* lane,
    const stygmergic_configuration& acq) const {
    auto dpos = mc_sensing->dpos2D();
  auto rpos = mc_sensing->rpos2D();
  auto* ct = mc_perception->nearest_ct();
  std::vector<rmath::vector2d> path = {rpos};

  /*
   * \todo Right now this assumes cube blocks, and is only correct for
   * such. This will be fixed once the most basic construction has been
   * validated.
   */
  if (rmath::radians::kZERO == lane->orientation()) {
    if (stygmergic_configuration::ekLANE_EMPTY == acq ||
        stygmergic_configuration::ekLANE_GAP_INGRESS == acq) {
      rmath::vector2z forward1(dpos.x() - 1 *ct->unit_dim_factor(), dpos.y());
      path.push_back(rmath::zvec2dvec(forward1,
                                      mc_perception->arena_resolution().v()));
    } else if (stygmergic_configuration::ekLANE_FILLED == acq) {
      /* no further waypoints needed */
    } else if (stygmergic_configuration::ekLANE_GAP_EGRESS == acq) {
      rmath::vector2z forward1(dpos.x() - 1 *ct->unit_dim_factor(), dpos.y());
      rmath::vector2z right1(dpos.x() - 1 *ct->unit_dim_factor(),
                             dpos.y() + 1 *ct->unit_dim_factor());
      path.push_back(rmath::zvec2dvec(forward1,
                                      mc_perception->arena_resolution().v()));
      path.push_back(rmath::zvec2dvec(right1,
                                      mc_perception->arena_resolution().v()));
    }
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation()) {
    if (stygmergic_configuration::ekLANE_EMPTY == acq ||
        stygmergic_configuration::ekLANE_GAP_INGRESS == acq) {
      rmath::vector2z forward1(dpos.x(), dpos.y() - 1 *ct->unit_dim_factor());
      path.push_back(rmath::zvec2dvec(forward1,
                                      mc_perception->arena_resolution().v()));
    } else if (stygmergic_configuration::ekLANE_FILLED == acq) {
      /* no further waypoints needed */
    } else if (stygmergic_configuration::ekLANE_GAP_EGRESS == acq) {
      rmath::vector2z forward1(dpos.x(), dpos.y() - 1 *ct->unit_dim_factor());
      rmath::vector2z right1(dpos.x() - 1 *ct->unit_dim_factor(),
                             dpos.y() - 1 * ct->unit_dim_factor());
      path.push_back(rmath::zvec2dvec(forward1,
                                      mc_perception->arena_resolution().v()));
      path.push_back(rmath::zvec2dvec(right1,
                                      mc_perception->arena_resolution().v()));
    }
  }
  return path;
} /* operator()() */

NS_END(fsm, silicon);
