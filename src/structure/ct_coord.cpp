/**
 * \file ct_coord.cpp
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
#include "silicon/structure/ct_coord.hpp"

#include "silicon/structure/structure3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::vector3z to_vcoord(const ct_coord& coord, const structure3D* ct) {
  if (coord_relativity::ekRORIGIN == coord.relative_to) {
    return (ct->rorigind() - ct->vorigind()) + coord.offset;
  } else {
    return coord.offset;
  }
} /* to_vcoord() */

rmath::vector3z to_rcoord(const ct_coord& coord, const structure3D* ct) {
  if (coord_relativity::ekVORIGIN == coord.relative_to) {
    return coord.offset - (ct->rorigind() - ct->vorigind());
  } else {
    return coord.offset;
  }
} /* to_rcoord() */

NS_END(structure, silicon);
