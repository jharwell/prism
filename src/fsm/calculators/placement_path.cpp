/**
 * \file placement_path.cpp
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
#include "silicon/fsm/calculators/placement_path.hpp"

#include "rcppsw/math/radians.hpp"

#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

#include "silicon/controller/perception/builder_perception_subsystem.hpp"
#include "silicon/repr/construction_lane.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, fsm, calculators);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
placement_path::placement_path(
    const csubsystem::sensing_subsystemQ3D* sensing,
    const scperception::builder_perception_subsystem* perception)
    : ER_CLIENT_INIT("silicon.fsm.calculator.placement_path"),
      mc_sensing(sensing),
      mc_perception(perception) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::vector<rmath::vector2d> placement_path::operator()(
    const srepr::construction_lane* lane,
    const stygmergic_configuration& acq) const {
  auto rpos = mc_sensing->rpos2D();
  auto* ct = mc_perception->nearest_ct();
  std::vector<rmath::vector2d> path = {rpos};
  double cell_size = ct->block_unit_dim();

  /*
   * \todo Right now this assumes cube blocks, and is only correct for
   * such. This will be fixed once the most basic construction has been
   * validated.
   */
  if (rmath::radians::kZERO == lane->orientation()) {
    rmath::vector2d forward1(rpos.x() - 1 * cell_size * 1.5, rpos.y());
    rmath::vector2d right1(rpos.x() - 1 * cell_size * 1.5,
                           rpos.y() + 1 * cell_size);
    if (stygmergic_configuration::ekLANE_EMPTY == acq ||
        stygmergic_configuration::ekLANE_GAP_INGRESS == acq) {
      path.push_back(forward1);
    } else if (stygmergic_configuration::ekLANE_FILLED == acq) {
      /* no further waypoints needed */
    } else if (stygmergic_configuration::ekLANE_GAP_EGRESS == acq) {
      path.push_back(forward1);
      path.push_back(right1);
    }
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation()) {
    rmath::vector2d forward1(rpos.x(), rpos.y() - 1 * cell_size * 1.5);
    rmath::vector2d right1(rpos.x() - 1 * cell_size,
                           rpos.y() - 1 * cell_size * 1.5);
    if (stygmergic_configuration::ekLANE_EMPTY == acq ||
        stygmergic_configuration::ekLANE_GAP_INGRESS == acq) {
      path.push_back(forward1);
    } else if (stygmergic_configuration::ekLANE_FILLED == acq) {
      /* no further waypoints needed */
    } else if (stygmergic_configuration::ekLANE_GAP_EGRESS == acq) {
      path.push_back(forward1);
      path.push_back(right1);
    }
  } else if (rmath::radians::kPI == lane->orientation()) {
    rmath::vector2d forward1(rpos.x() + 1 * cell_size * 1.5, rpos.y());
    rmath::vector2d right1(rpos.x() + 1 * cell_size * 1.5,
                           rpos.y() - 1 * cell_size);
    if (stygmergic_configuration::ekLANE_EMPTY == acq ||
        stygmergic_configuration::ekLANE_GAP_INGRESS == acq) {
      path.push_back(forward1);
    } else if (stygmergic_configuration::ekLANE_FILLED == acq) {
      /* no further waypoints needed */
    } else if (stygmergic_configuration::ekLANE_GAP_EGRESS == acq) {
      path.push_back(forward1);
      path.push_back(right1);
    }
  } else if (rmath::radians::kTHREE_PI_OVER_TWO == lane->orientation()) {
    rmath::vector2d forward1(rpos.x(), rpos.y() + 1 * cell_size * 1.5);
    rmath::vector2d right1(rpos.x() + 1 * cell_size,
                           rpos.y() + 1 * cell_size * 1.5);
    if (stygmergic_configuration::ekLANE_EMPTY == acq ||
        stygmergic_configuration::ekLANE_GAP_INGRESS == acq) {
      path.push_back(forward1);
    } else if (stygmergic_configuration::ekLANE_FILLED == acq) {
      /* no further waypoints needed */
    } else if (stygmergic_configuration::ekLANE_GAP_EGRESS == acq) {
      path.push_back(forward1);
      path.push_back(right1);
    }
  } else {
    ER_FATAL_SENTINEL("Bad orientation: '%s'",
                      rcppsw::to_string(lane->orientation()).c_str());
  }
  return path;
} /* operator()() */

NS_END(calculators, fsm, silicon);
