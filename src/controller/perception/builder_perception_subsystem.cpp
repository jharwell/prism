/**
 * \file builder_perception_subsystem.cpp
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
#include "silicon/controller/perception/builder_perception_subsystem.hpp"

#include "silicon/structure/structure3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, controller, perception);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void builder_perception_subsystem::update() {
  mc_target = los()->target();
} /* update() */

const rtypes::discretize_ratio& builder_perception_subsystem::grid_resolution(void) const {
  return mc_target->resolution();
} /* grid_resolution() */

rmath::ranged builder_perception_subsystem::structure_xrange(void) const {
  return mc_target->xrange();
} /* structure_xrange() */

rmath::ranged builder_perception_subsystem::structure_yrange(void) const {
  return mc_target->yrange();
} /* structure_yrange() */

NS_END(perception, controller, silicon);
