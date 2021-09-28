/**
 * \file placement_path.cpp
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
#include "prism/fsm/calculators/placement_path.hpp"

#include "rcppsw/math/radians.hpp"

#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

#include "prism/controller/perception/builder_perception_subsystem.hpp"
#include "prism/repr/construction_lane.hpp"
#include "prism/gmt/utils.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, fsm, calculators);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
placement_path::placement_path(
    const csubsystem::sensing_subsystemQ3D* sensing,
    const pcperception::builder_perception_subsystem* perception)
    : ER_CLIENT_INIT("prism.fsm.calculator.placement_path"),
      mc_sensing(sensing),
      mc_perception(perception) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::vector<rmath::vector2d>
placement_path::operator()(const prepr::construction_lane* lane,
                           const prepr::fs_acq_result& acq) const {
  auto rpos = mc_sensing->rpos2D();
  const auto* ct = mc_perception->nearest_ct();
  std::vector<rmath::vector2d> path = { rpos };
  double cell_size = ct->block_unit_dim().v();

  ER_ASSERT(pgmt::orientation_valid(lane->orientation()),
            "Bad orientation: '%s'",
            rcppsw::to_string(lane->orientation()).c_str());

  /*
   * \todo Right now this assumes cube blocks, and is only correct for
   * such. This will be fixed once the most basic construction has been
   * validated.
   */
  bool in_vshell = false;
  if (rmath::radians::kZERO == lane->orientation() ||
      rmath::radians::kPI == lane->orientation()) {
    in_vshell = ct->xrspan(true).contains(rpos.x());
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation() ||
             rmath::radians::kTHREE_PI_OVER_TWO == lane->orientation()) {
    in_vshell = ct->yrspan(true).contains(rpos.y());
  }


  rmath::vector2d forward2p25;
  rmath::vector2d forward2;
  rmath::vector2d forward1p75;
  rmath::vector2d forward1p5;
  rmath::vector2d right1_for1p5;
  rmath::vector2d right1_for1p75;
  rmath::vector2d right1_for2;
  rmath::vector2d right1_for2p25;

  /*
   * In order to make block placements look "mostly" physical, we need to move
   * forward by 2 OR 2.25 OR 1.5 cells from our current position, due to the
   * truncation used when robot position is converted from real -> discrete ->
   * CT coordinates. We don't have to move at all to get algorithmic
   * correctness, but it help the simulations to look nice. The {2,2.25,1.5}
   * values are strongly tied to the value of \ref
   * palgorithm::constants::kCT_FS_LOOKAHEAD_MAX_CELLS and the size of the robot
   * LOS. If chose change value, then these values will have to be updated as
   * well.
   *
   * LANE_EMPTY is the only one that requires 1.5 cells because we detect this
   * configuration using the EDGE of our LOS, rather than what cells it
   * contains.
   *
   * \todo Calculate values programmatically rather than having a bunch of magic
   * numbers here.
   */
  if (rmath::radians::kZERO == lane->orientation()) {
    forward2p25 = {rpos.x() + cell_size * 2.25, rpos.y()};
    forward2 = {rpos.x() + cell_size * 2, rpos.y()};
    forward1p75 = {rpos.x() + cell_size * 1.75, rpos.y()};
    forward1p5 = {rpos.x() + cell_size * 1.5, rpos.y()};
    right1_for1p5 = {rpos.x() + cell_size * 1.5, rpos.y() - cell_size};
    right1_for1p75 = {rpos.x() + cell_size * 1.75, rpos.y() - cell_size};
    right1_for2 = {rpos.x() + cell_size * 2, rpos.y() - cell_size};
    right1_for2p25 = {rpos.x() + cell_size * 2.25, rpos.y() - cell_size};
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation()) {
    forward2p25 = {rpos.x(), rpos.y() + cell_size * 2.25};
    forward2 = {rpos.x(), rpos.y() + cell_size * 2};
    forward1p75 = {rpos.x(), rpos.y() + cell_size * 1.75};
    forward1p5 = {rpos.x(), rpos.y() + cell_size * 1.5};
    right1_for1p5 = {rpos.x() + cell_size, rpos.y() + cell_size * 1.5};
    right1_for1p75 = {rpos.x() + cell_size, rpos.y() + cell_size * 1.75};
    right1_for2 = {rpos.x() + cell_size, rpos.y() + cell_size * 2};
    right1_for2p25 = {rpos.x() + cell_size, rpos.y() + cell_size * 2.25};
  } else if (rmath::radians::kPI == lane->orientation()) {
    forward2p25 = {rpos.x() - cell_size * 2.25, rpos.y()};
    forward2 = {rpos.x() - cell_size * 2, rpos.y()};
    forward1p75 = {rpos.x() - cell_size * 1.75, rpos.y()};
    forward1p5 = {rpos.x() - cell_size * 1.5, rpos.y()};
    right1_for1p5 = {rpos.x() - cell_size * 1.5, rpos.y() + cell_size};
    right1_for1p75 = {rpos.x() - cell_size * 1.75, rpos.y() + cell_size};
    right1_for2 = {rpos.x() - cell_size * 2, rpos.y() + cell_size};
    right1_for2p25 = {rpos.x() - cell_size * 2.25, rpos.y() + cell_size};
  } else if (rmath::radians::kTHREE_PI_OVER_TWO == lane->orientation()) {
    forward2p25 = {rpos.x(), rpos.y() - cell_size * 2.25};
    forward2 = {rpos.x(), rpos.y() - cell_size * 2};
    forward1p75 = {rpos.x(), rpos.y() - cell_size * 1.75};
    forward1p5 = {rpos.x(), rpos.y() - cell_size * 1.5};
    right1_for1p5 = {rpos.x() - cell_size, rpos.y() - cell_size * 1.5 };
    right1_for1p75 = {rpos.x() - cell_size, rpos.y() - cell_size * 1.75 };
    right1_for2 = {rpos.x() - cell_size, rpos.y() - cell_size * 2 };
    right1_for2p25 = {rpos.x() - cell_size, rpos.y() - cell_size * 2.25 };
  }

  ER_ASSERT(2 == acq.lookahead || 3 == acq.lookahead, "Lookhead == 1 ?");
  if (prepr::fs_configuration::ekLANE_EMPTY == acq.configuration) {
    path.push_back(forward1p5);
  } else if (prepr::fs_configuration::ekLANE_FILLED == acq.configuration) {
    if (2 == acq.lookahead) {
      path.push_back(forward1p5);
    } else {
      path.push_back(forward1p75);
    }
  } else if (prepr::fs_configuration::ekLANE_GAP_INGRESS == acq.configuration) {
    if (2 == acq.lookahead) {
      path.push_back(forward1p5);
    } else {
      path.push_back(forward2);
    }
  } else if (prepr::fs_configuration::ekLANE_GAP_EGRESS == acq.configuration) {
    if (2 == acq.lookahead) {
      path.push_back(forward1p75);
      path.push_back(right1_for1p75);
    } else {
      path.push_back(forward2p25);
      path.push_back(right1_for2p25);
    }
  }

  ER_INFO("Calculated placement path for fs=%d, %zu waypoints",
          rcppsw::as_underlying(acq.configuration),
          path.size());

  return path;
} /* operator()() */

NS_END(calculators, fsm, prism);
