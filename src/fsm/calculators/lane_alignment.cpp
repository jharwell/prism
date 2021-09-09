/**
 * \file lane_alignment.cpp
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
#include "prism/fsm/calculators/lane_alignment.hpp"

#include "rcppsw/math/radians.hpp"
#include "rcppsw/math/degrees.hpp"

#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

#include "prism/repr/construction_lane.hpp"
#include "prism/gmt/utils.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, fsm, calculators);

/*******************************************************************************
 * Class Constants
 ******************************************************************************/
const rmath::radians lane_alignment::kAZIMUTH_TOL =
    rmath::radians(rmath::degrees(15));
const rtypes::spatial_dist lane_alignment::kTRAJECTORY_ORTHOGONAL_TOL =
    rtypes::spatial_dist(0.2);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
lane_alignment::ret_type
lane_alignment::operator()(const prepr::construction_lane* lane) const {
  return { verify_pos(lane->geometry().ingress_pt().to_2D(), lane->orientation()),
           verify_pos(lane->geometry().egress_pt().to_2D(), lane->orientation()),
           verify_azimuth(lane->orientation()) };
} /* operator()() */

bool lane_alignment::verify_pos(const rmath::vector2d& lane_point,
                                const rmath::radians& orientation) const {
  ER_ASSERT(pgmt::orientation_valid(orientation),
            "Bad orientation: '%s'",
            rcppsw::to_string(orientation).c_str());

  auto dist_diff = mc_sensing->rpos2D() - lane_point;
  if (rmath::radians::kZERO == orientation ||
      rmath::radians::kPI == orientation) {
    return (rtypes::spatial_dist::make(dist_diff.y()) <=
            kTRAJECTORY_ORTHOGONAL_TOL);
  } else if (rmath::radians::kPI_OVER_TWO == orientation ||
             rmath::radians::kTHREE_PI_OVER_TWO == orientation) {
    return (rtypes::spatial_dist::make(dist_diff.x()) <=
            kTRAJECTORY_ORTHOGONAL_TOL);
  }
  return false;
} /* verify_pos() */

bool lane_alignment::verify_azimuth(const rmath::radians& orientation) const {
  rmath::radians angle_diff;
  ER_ASSERT(pgmt::orientation_valid(orientation),
            "Bad orientation: '%s'",
            rcppsw::to_string(orientation).c_str());

  if (rmath::radians::kZERO == orientation) {
    angle_diff =
        (rmath::radians::kZERO - mc_sensing->azimuth()).signed_normalize();
  } else if (rmath::radians::kPI_OVER_TWO == orientation) {
    angle_diff =
        (rmath::radians::kPI_OVER_TWO - mc_sensing->azimuth()).signed_normalize();
  } else if (rmath::radians::kPI == orientation) {
    angle_diff = (rmath::radians::kPI - mc_sensing->azimuth()).signed_normalize();
  } else if (rmath::radians::kTHREE_PI_OVER_TWO == orientation) {
    angle_diff = (rmath::radians::kTHREE_PI_OVER_TWO - mc_sensing->azimuth())
                 .signed_normalize();
  }

  return angle_diff <= kAZIMUTH_TOL;
} /* lane_alignment_verify_azimuth() */

NS_END(calculators, fsm, prism);
