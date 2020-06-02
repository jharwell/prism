/**
 * \file fs_path_calculator.cpp
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
#include "silicon/fsm/fs_path_calculator.hpp"

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
fs_path_calculator::fs_path_calculator(
    const csubsystem::sensing_subsystemQ3D* sensing)
    : ER_CLIENT_INIT("silicon.fsm.fs_path_calculator"), mc_sensing(sensing) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::vector<rmath::vector2d> fs_path_calculator::operator()(
    const srepr::construction_lane* lane) const {
  /* 1st point: robot's current position */
  std::vector<rmath::vector2d> path = {mc_sensing->rpos2D()};

  if (rmath::radians::kZERO == lane->orientation()) {
    /* 2nd point: middle of the chosen lane */
    path.push_back({0.0, lane->ingress().y()});

    /* 3rd point: Back of the structure */
    path.push_back({0.0, lane->ingress().y()});
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation()) {
    /* 2nd point: middle of the chosen lane */
    path.push_back({lane->ingress().x(), 0.0});

    /* 3rd point: Back of the structure */
    path.push_back({lane->ingress().x(), 0.0});
  }
  return path;
} /* operator()() */

NS_END(fsm, silicon);
