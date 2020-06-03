/**
 * \file egress_path.cpp
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
#include "silicon/fsm/calculators/egress_path.hpp"

#include "rcppsw/math/radians.hpp"

#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

#include "silicon/repr/construction_lane.hpp"
#include "silicon/controller/perception/builder_perception_subsystem.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, fsm, calculators);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
egress_path::egress_path(
    const csubsystem::sensing_subsystemQ3D* sensing,
    const scperception::builder_perception_subsystem* perception,
    rmath::rng* rng)
    : ER_CLIENT_INIT("silicon.fsm.calculator.egress_path"),
      mc_sensing(sensing),
      mc_perception(perception),
      m_rng(rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::vector<rmath::vector2d> egress_path::operator()(
    const srepr::construction_lane* lane) const {
  auto pos = mc_sensing->rpos2D();
  auto* ct = mc_perception->nearest_ct();

  /* 1st point: robot's current position */
  std::vector<rmath::vector2d> path = {pos};

  double x, y;
  if (rmath::radians::kZERO == lane->orientation()) {
    x = m_rng->uniform(ct->xrange().ub() +
                       ct->block_unit_dim() * kNEST_PADDING,
                       mc_perception->arena_xrange().ub() -
                       ct->block_unit_dim() * kNEST_PADDING);
    y = lane->egress().y();
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation()) {
    x = lane->egress().x();
    y = m_rng->uniform(ct->yrange().ub() +
                       ct->block_unit_dim() * kNEST_PADDING,
                       mc_perception->arena_yrange().ub() -
                       ct->block_unit_dim() * kNEST_PADDING);
  } else if (rmath::radians::kPI == lane->orientation()) {
    x = m_rng->uniform(mc_perception->arena_xrange().lb() +
                       ct->block_unit_dim() * kNEST_PADDING,
                       ct->xrange().lb() - ct->block_unit_dim() * kNEST_PADDING);
    y = lane->egress().y();
  } else if (rmath::radians::kTHREE_PI_OVER_TWO == lane->orientation()) {
    x = lane->egress().x();
    y = m_rng->uniform(mc_perception->arena_yrange().lb() +
                       ct->block_unit_dim() * kNEST_PADDING,
                       ct->yrange().lb() -
                       ct->block_unit_dim() * kNEST_PADDING);
  } else {
    ER_FATAL_SENTINEL("Bad orientation: '%s'",
                      rcppsw::to_string(lane->orientation()).c_str());
  }

  /* 2nd point: just outside structure ingress/egress face boundary  */
  path.push_back({x, y});
  return path;
} /* operator()() */

NS_END(calculators, fsm, silicon);
