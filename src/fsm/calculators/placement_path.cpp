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
                           const prepr::fs_configuration& acq) const {
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

  rmath::vector2d forward1p25;
  rmath::vector2d forward1p5;
  rmath::vector2d forward1;
  rmath::vector2d right1;
  if (rmath::radians::kZERO == lane->orientation()) {
    forward1p25 = {rpos.x() + cell_size * 1.25, rpos.y()};
    forward1p25 = {rpos.x() + cell_size * 1.5, rpos.y()};
    forward1 = {rpos.x() + cell_size, rpos.y()};
    right1 = {rpos.x() + cell_size * 1.25, rpos.y() - cell_size};
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation()) {
    forward1p25 = {rpos.x(), rpos.y() + cell_size * 1.25};
    forward1p5 = {rpos.x(), rpos.y() + cell_size * 1.5};
    forward1 = {rpos.x(), rpos.y() + cell_size};
    right1 = {rpos.x() + cell_size, rpos.y() + cell_size * 1.25};
  } else if (rmath::radians::kPI == lane->orientation()) {
    forward1p25 = {rpos.x() - cell_size * 1.25, rpos.y()};
    forward1p5 = {rpos.x() - cell_size * 1.5, rpos.y()};
    forward1 = {rpos.x() - cell_size, rpos.y()};
    right1 = {rpos.x() - cell_size * 1.25, rpos.y() + cell_size};
  } else if (rmath::radians::kTHREE_PI_OVER_TWO == lane->orientation()) {
    forward1p25 = {rpos.x(), rpos.y() - cell_size * 1.25};
    forward1p5 = {rpos.x(), rpos.y() - cell_size * 1.5};
    forward1 = {rpos.x(), rpos.y() - cell_size};
    right1 = {rpos.x() - cell_size, rpos.y() - cell_size * 1.25 };
  }

  /*
   * In order to make block placements look "mostly" physical, we need to move
   * forward by 1 OR 1.25 OR 1.5 cells from our current position, due to the
   * truncation used when robot position is converted from real -> discrete ->
   * CT coordinates. We don't have to move at all to get algorithmic
   * correctness, but it help the simulations to look nice.
   *
   * LANE_EMPTY is the only one that requires 1.5 cells because we detect this
   * configuration using the EDGE of our LOS, rather than what cells it
   * contains.
   */
  if (prepr::fs_configuration::ekLANE_EMPTY == acq) {
    path.push_back(forward1p5);
  } else if (prepr::fs_configuration::ekLANE_FILLED == acq) {
    path.push_back(forward1);
  } else if (prepr::fs_configuration::ekLANE_GAP_INGRESS == acq) {
    path.push_back(forward1);
  } else if (prepr::fs_configuration::ekLANE_GAP_EGRESS == acq) {
    path.push_back(forward1p25);
    path.push_back(right1);
  }

  ER_INFO("Calculated placement path for fs=%d, %zu waypoints",
          rcppsw::as_underlying(acq),
          path.size());

  return path;
} /* operator()() */

NS_END(calculators, fsm, prism);
