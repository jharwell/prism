/**
 * \file ingress_lane_path.cpp
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
#include "silicon/fsm/calculators/ingress_lane_path.hpp"

#include "rcppsw/math/radians.hpp"

#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

#include "silicon/repr/construction_lane.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, fsm, calculators);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
ingress_lane_path::ingress_lane_path(
    const csubsystem::sensing_subsystemQ3D* sensing)
    : ER_CLIENT_INIT("silicon.fsm.calculator.ingress_lane_path"),
      mc_sensing(sensing) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::vector<rmath::vector2d>
ingress_lane_path::operator()(const srepr::construction_lane* lane) const {
  auto pos = mc_sensing->rpos2D();

  /* 1st point: robot's current location */
  std::vector<rmath::vector2d> path{ pos };

  if (rmath::radians::kZERO == lane->orientation() ||
      rmath::radians::kPI == lane->orientation()) {
    /* 2nd point: get aligned with middle of ingress lane */
    path.push_back({ pos.x(), lane->ingress().y() });

    /* 3rd point: ingress */
    path.push_back(lane->ingress().to_2D());
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation() ||
             rmath::radians::kTHREE_PI_OVER_TWO == lane->orientation()) {
    /* 2nd point: get aligned with ingress lane */
    path.push_back({ lane->ingress().x(), pos.y() });

    /* 3rd point: ingress */
    path.push_back(lane->ingress().to_2D());
  } else {
    ER_FATAL_SENTINEL("Bad orientation: '%s'",
                      rcppsw::to_string(lane->orientation()).c_str());
  }
  return path;
} /* operator()() */

NS_END(calculators, fsm, silicon);
