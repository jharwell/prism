/**
 * \file egress_lane_path.cpp
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
#include "silicon/fsm/calculators/egress_lane_path.hpp"

#include "rcppsw/math/radians.hpp"

#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

#include "silicon/fsm/builder_util_fsm.hpp"
#include "silicon/fsm/calculators/lane_alignment.hpp"
#include "silicon/repr/construction_lane.hpp"
#include "silicon/structure/utils.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, fsm, calculators);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
egress_lane_path::egress_lane_path(const csubsystem::sensing_subsystemQ3D* sensing)
    : ER_CLIENT_INIT("silicon.fsm.calculator.egress_lane_path"),
      mc_sensing(sensing) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::vector<rmath::vector2d>
egress_lane_path::operator()(const srepr::construction_lane* lane) const {
  auto rpos = mc_sensing->rpos2D();
  auto egress_pt = lane->geometry().egress_pt();
  std::vector<rmath::vector2d> path = { rpos };

  ER_ASSERT(sstructure::orientation_valid(lane->orientation()),
            "Bad orientation: '%s'",
            rcppsw::to_string(lane->orientation()).c_str());

  auto alignment = calculators::lane_alignment(mc_sensing)(lane);
  /*
     * We only have a path to the ingress lane if we are not currently in it
     * (i.e., placed our block at the back of the ingress lane).
     */
  if ((rmath::radians::kZERO == lane->orientation() ||
       rmath::radians::kPI == lane->orientation())) {
    if (!alignment.egress) {
      path.push_back({ rpos.x(), egress_pt.y() });
    }
  } else if ((rmath::radians::kPI_OVER_TWO == lane->orientation() ||
              rmath::radians::kTHREE_PI_OVER_TWO == lane->orientation())) {
    if (!alignment.egress) {
      path.push_back({ egress_pt.x(), rpos.y() });
    }
  }
  return path;
} /* operator()() */

NS_END(calculators, fsm, silicon);
