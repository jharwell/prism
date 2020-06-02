/**
 * \file placement_intent_calculator.cpp
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
#include "silicon/fsm/placement_intent_calculator.hpp"

#include "rcppsw/math/radians.hpp"

#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

#include "silicon/controller/perception/builder_perception_subsystem.hpp"
#include "silicon/repr/construction_lane.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
placement_intent_calculator::placement_intent_calculator(
    const csubsystem::sensing_subsystemQ3D* sensing,
    const scperception::builder_perception_subsystem* perception)
    : ER_CLIENT_INIT("silicon.fsm.placement_intent_calculator"),
      mc_sensing(sensing),
      mc_perception(perception) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
block_placer::placement_intent placement_intent_calculator::operator()(
    const srepr::construction_lane* lane) const {
  auto pos = mc_sensing->dpos3D();
  auto* ct = mc_perception->nearest_ct();
  block_placer::placement_intent ret;

  /*
   * \todo Right now this assumes cube blocks, and is only correct for
   * such. This will be fixed once the most basic construction has been
   * validated.
   */
  if (rmath::radians::kZERO == lane->orientation()) {
    /*
     * This is the absolute location of the block placement intent in the
     * ARENA, which is what the origind() function also returns for the
     * structure. However, to get to the relative intent within the target
     * structure, simply subtracting the origind() is not enough if the grid
     * resolution for the structure is larger than that for the arena.
     */
    auto intent_abs = pos - rmath::vector3z::X * ct->unit_dim_factor();
    ret = {(intent_abs - ct->origind()) / ct->unit_dim_factor(),
           rmath::radians::kZERO};
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation()) {
    auto intent_abs = pos - rmath::vector3z::Y * ct->unit_dim_factor();
    ret = {(intent_abs - ct->origind()) / ct->unit_dim_factor(),
           rmath::radians::kZERO};
  } else {
    ER_FATAL_SENTINEL("Bad lane orientation '%s'",
                      rcppsw::to_string(lane->orientation()).c_str());
  }
  ER_INFO("Calculated placement intent: %s@%s",
          rcppsw::to_string(ret.site).c_str(),
          rcppsw::to_string(ret.orientation).c_str());
  return ret;
} /* operator()() */

NS_END(fsm, silicon);
