/**
 * \file egress_path.cpp
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
#include "prism/fsm/calculators/egress_path.hpp"

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
egress_path::egress_path(
    const csubsystem::sensing_subsystemQ3D* sensing,
    const pcperception::builder_perception_subsystem* perception,
    rmath::rng* rng)
    : ER_CLIENT_INIT("prism.fsm.calculator.egress_path"),
      mc_sensing(sensing),
      mc_perception(perception),
      m_rng(rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::vector<rmath::vector2d>
egress_path::operator()(const prepr::construction_lane* lane) const {
  auto pos = mc_sensing->rpos2D();
  const auto* ct = mc_perception->nearest_ct();
  auto egress_pt = lane->geometry().egress_pt();

  ER_ASSERT(pgmt::orientation_valid(lane->orientation()),
            "Bad orientation: '%s'",
            rcppsw::to_string(lane->orientation()).c_str());

  /* 1st point: robot's current position */
  std::vector<rmath::vector2d> path = { pos };

  double x, y;
  if (rmath::radians::kZERO == lane->orientation()) {
    x = m_rng->uniform(mc_perception->arena_xrspan().lb() +
                       ct->block_unit_dim().v(),
                       ct->xrspan().lb() - ct->block_unit_dim().v() * kNEST_PADDING);
    y = egress_pt.y();
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation()) {
    x = egress_pt.x();
    y = m_rng->uniform(mc_perception->arena_yrspan().lb() +
                       ct->block_unit_dim().v(),
                       ct->yrspan().lb() - ct->block_unit_dim().v() * kNEST_PADDING);
  } else if (rmath::radians::kPI == lane->orientation()) {
    x = m_rng->uniform(ct->xrspan().ub() + ct->block_unit_dim().v() * kNEST_PADDING,
                       mc_perception->arena_xrspan().ub() - ct->block_unit_dim().v());
    y = egress_pt.y();
  } else if (rmath::radians::kTHREE_PI_OVER_TWO == lane->orientation()) {
    x = egress_pt.x();
    y = m_rng->uniform(ct->yrspan().ub() + ct->block_unit_dim().v() * kNEST_PADDING,
                       mc_perception->arena_yrspan().ub() - ct->block_unit_dim().v());
  }

  /* 2nd point: just outside structure ingress/egress face boundary  */
  path.push_back({ x, y });
  return path;
} /* operator()() */

NS_END(calculators, fsm, prism);
