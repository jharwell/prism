/**
 * \file ingress_path.cpp
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
#include "prism/fsm/calculators/ingress_path.hpp"

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
ingress_path::ingress_path(
    const csubsystem::sensing_subsystemQ3D* sensing,
    const pcperception::builder_perception_subsystem* perception)
    : ER_CLIENT_INIT("prism.fsm.calculator.ingress_path"),
      mc_sensing(sensing),
      mc_perception(perception) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::vector<rmath::vector2d>
ingress_path::operator()(const prepr::construction_lane* lane) const {
  const auto* ct = mc_perception->nearest_ct();
  auto ingress_pt = lane->geometry().ingress_pt();

  /* 1st point: robot's current position */
  std::vector<rmath::vector2d> path = { mc_sensing->rpos2D() };

  ER_ASSERT(pgmt::orientation_valid(lane->orientation()),
            "Bad orientation: '%s'",
            rcppsw::to_string(lane->orientation()).c_str());

  if (rmath::radians::kZERO == lane->orientation()) {
    /* 2nd point: Back of the structure */
    path.emplace_back(ct->xrspan(true).ub(), ingress_pt.y());
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation()) {
    /* 2nd point: Back of the structure */
    path.emplace_back(ingress_pt.x(), ct->yrspan(true).ub());
  } else if (rmath::radians::kPI == lane->orientation()) {
    /* 2nd point: Back of the structure */
    path.emplace_back(ct->xrspan(true).lb(), ingress_pt.y());
  } else if (rmath::radians::kTHREE_PI_OVER_TWO == lane->orientation()) {
    /* 2nd point: Back of the structure */
    path.emplace_back(ingress_pt.x(), ct->yrspan(true).lb());
  }

  return path;
} /* operator()() */

NS_END(calculators, fsm, prism);
