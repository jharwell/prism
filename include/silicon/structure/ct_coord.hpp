/**
 * \file ct_coord.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_CT_COORD_HPP_
#define INCLUDE_SILICON_STRUCTURE_CT_COORD_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector3.hpp"

#include "silicon/silicon.hpp"
#include "coord_relativity.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure);

class structure3D;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief Representation of the location of a cell within a \ref
 * structure3D. Because of the real/virtual origin, offsets within the 3D grid
 * need to be tagged with what they are relative to, to make it VERY clear what
 * the \a implied origin is when an offset is passed to a different
 * function/class/etc.*
 */
struct ct_coord {
  rmath::vector3z offset;
  coord_relativity relative_to;
};

rmath::vector3z to_vcoord(const ct_coord& coord, const structure3D* ct);
rmath::vector3z to_rcoord(const ct_coord& coord, const structure3D* ct);

NS_END(structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_CT_COORD_HPP_ */
