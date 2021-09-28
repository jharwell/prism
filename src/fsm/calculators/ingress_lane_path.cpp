/**
 * \file ingress_lane_path.cpp
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
#include "prism/fsm/calculators/ingress_lane_path.hpp"

#include "rcppsw/math/radians.hpp"

#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

#include "prism/repr/construction_lane.hpp"
#include "prism/gmt/utils.hpp"
#include "prism/controller/perception/builder_perception_subsystem.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, fsm, calculators);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
ingress_lane_path::ingress_lane_path(
    const csubsystem::sensing_subsystemQ3D* sensing,
    const pcperception::builder_perception_subsystem* perception)
    : ER_CLIENT_INIT("prism.fsm.calculator.ingress_lane_path"),
      mc_sensing(sensing),
      mc_perception(perception) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::vector<rmath::vector2d>
ingress_lane_path::operator()(const prepr::construction_lane* lane) const {
  auto pos = mc_sensing->rpos2D();
  auto ingress_pt = lane->geometry().ingress_pt();

  ER_ASSERT(pgmt::orientation_valid(lane->orientation()),
            "Bad orientation: '%s'",
            rcppsw::to_string(lane->orientation()).c_str());

  double cell_size = mc_perception->nearest_ct()->block_unit_dim().v();

  /* 1st point: robot's current location */
  std::vector<rmath::vector2d> path{ pos };

  if (rmath::radians::kZERO == lane->orientation() ||
      rmath::radians::kPI == lane->orientation()) {
    /* 2nd point: get aligned with middle of ingress lane */
    path.push_back({ pos.x(), ingress_pt.y() });
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation() ||
             rmath::radians::kTHREE_PI_OVER_TWO == lane->orientation()) {
    /* 2nd point: get aligned with ingress lane */
    path.push_back({ ingress_pt.x(), pos.y() });

  }
  /*
   * The lane geometry has the ingress point at the center of the ingress cell,
   * and we need it to be on the edge of the cell/edge of the vshell, in order
   * to get proper placement of the last row of blocks in a construction lane.
   */
  rmath::vector2d ingress_corr;
  if (rmath::radians::kZERO == lane->orientation()) {
    ingress_corr = -rmath::vector2d::X * cell_size * 0.5;
  } else if (rmath::radians::kPI == lane->orientation()) {
    ingress_corr = rmath::vector2d::X * cell_size * 0.5;
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation()) {
    ingress_corr = -rmath::vector2d::Y * cell_size * 0.5;
  } else if (rmath::radians::kTHREE_PI_OVER_TWO == lane->orientation()) {
    ingress_corr = rmath::vector2d::Y * cell_size * 0.5;
  }

  /* 3rd point: ingress */
  path.push_back(ingress_pt.to_2D() + ingress_corr);

  return path;
} /* operator()() */

NS_END(calculators, fsm, prism);
