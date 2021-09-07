/**
 * \file ct_approach.cpp
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
#include "silicon/fsm/calculators/ct_approach.hpp"

#include "rcppsw/math/radians.hpp"

#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

#include "silicon/repr/construction_lane.hpp"
#include "silicon/structure/utils.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, fsm, calculators);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
ct_approach::ct_approach(const csubsystem::sensing_subsystemQ3D* sensing,
                         const rtypes::spatial_dist& lane_alignment_tol)
    : ER_CLIENT_INIT("silicon.fsm.calculators.ct_approach"),
      mc_lane_alignment_tol(lane_alignment_tol),
      mc_sensing(sensing) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
ct_approach::ct_approach_vector
ct_approach::operator()(const srepr::construction_lane* lane) const {
  ct_approach_vector ret;
  auto ingress_pt = lane->geometry().ingress_pt();

  ER_ASSERT(sstructure::orientation_valid(lane->orientation()),
            "Bad orientation: '%s'",
            rcppsw::to_string(lane->orientation()).c_str());

  auto angle_to_ingress =
      (mc_sensing->rpos2D() - ingress_pt.to_2D()).angle().unsigned_normalize();
  ret.ingress_angle =
      (lane->orientation() - angle_to_ingress).unsigned_normalize();

  if (rmath::radians::kZERO == lane->orientation()) {
    /* We are OK in X if we are on the -X side of the construction target */
    ret.x_ok = mc_sensing->rpos2D().x() <= ingress_pt.x();

    /*
     * We are OK in Y if we are (reasonably) close to the Y coordinate of the
     * ingress lane.
     */
    ret.orthogonal_dist =
        rtypes::spatial_dist::make(ingress_pt.y() - mc_sensing->rpos2D().y());
    ret.y_ok = ret.orthogonal_dist <= mc_lane_alignment_tol;
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation()) {
    /* We are OK in Y if we are on the -Y side of the construction target */
    ret.y_ok = mc_sensing->rpos2D().y() <= ingress_pt.y();

    /*
     * We are OK in X if we are (reasonably) close to the X coordinate of the
     * ingress lane.
     */
    ret.orthogonal_dist =
        rtypes::spatial_dist::make(ingress_pt.x() - mc_sensing->rpos2D().x());
    ret.x_ok = ret.orthogonal_dist <= mc_lane_alignment_tol;
  } else if (rmath::radians::kPI == lane->orientation()) {
    /* We are OK in X if we are on the +X side of the construction target */
    ret.x_ok = mc_sensing->rpos2D().x() >= ingress_pt.x();

    /*
     * We are OK in Y if we are (reasonably) close to the Y coordinate of the
     * ingress lane.
     */
    ret.orthogonal_dist =
        rtypes::spatial_dist::make(ingress_pt.y() - mc_sensing->rpos2D().y());
    ret.y_ok = ret.orthogonal_dist <= mc_lane_alignment_tol;

  } else if (rmath::radians::kTHREE_PI_OVER_TWO == lane->orientation()) {
    /* We are OK in Y if we are on the +Y side of the construction target */
    ret.y_ok = mc_sensing->rpos2D().y() >= ingress_pt.y();

    /*
     * We are OK in X if we are (reasonably) close to the X coordinate of the
     * ingress lane.
     */
    ret.orthogonal_dist =
        rtypes::spatial_dist::make(ingress_pt.x() - mc_sensing->rpos2D().x());
    ret.x_ok = ret.orthogonal_dist <= mc_lane_alignment_tol;
  }
  return ret;
} /* operator()() */

NS_END(calculators, fsm, silicon);
