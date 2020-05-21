/**
 * \file ct_approach_calculator.cpp
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
#include "silicon/fsm/ct_approach_calculator.hpp"

#include "rcppsw/math/radians.hpp"

#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

#include "silicon/repr/construction_lane.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
ct_approach_calculator::ct_approach_calculator(
    const csubsystem::sensing_subsystemQ3D* sensing,
    double lane_alignment_tol)
    : ER_CLIENT_INIT("silicon.fsm.ct_approach_calculator"),
      mc_lane_alignment_tol(lane_alignment_tol),
      mc_sensing(sensing) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
ct_approach_calculator::ct_approach_ret ct_approach_calculator::operator()(
    const srepr::construction_lane* lane) const {
  ct_approach_ret ret;
  if (rmath::radians::kZERO == lane->orientation()) {
    /* We are OK in X if we are on the +X side of the construction target */
    ret.x_ok = mc_sensing->rpos2D().x() >= lane->ingress().x();

    /*
     * We are OK in Y if we are (reasonably) close to the Y coordinate of the
     * ingress lane.
     */
    ret.orthogonal_dist = lane->ingress().y() - mc_sensing->rpos2D().y();
    ret.y_ok = std::fabs(ret.orthogonal_dist) <= mc_lane_alignment_tol;
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation()) {
    /* We are OK in Y if we are on the +Y side of the construction target */
    ret.y_ok = mc_sensing->rpos2D().y() >= lane->ingress().y();

    /*
     * We are OK in X if we are (reasonably) close to the X coordinate of the
     * ingress lane.
     */
    ret.orthogonal_dist = lane->ingress().x() - mc_sensing->rpos2D().x();
    ret.x_ok = std::fabs(ret.orthogonal_dist) <= mc_lane_alignment_tol;
  } else {
    ER_FATAL_SENTINEL("Bad orientation: '%s'",
                      rcppsw::to_string(lane->orientation()).c_str());
  }
  return ret;
} /* operator()() */

NS_END(fsm, silicon);
