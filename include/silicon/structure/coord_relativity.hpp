/**
 * \file coord_relativity.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_COORD_RELATIVITY_HPP_
#define INCLUDE_SILICON_STRUCTURE_COORD_RELATIVITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief Type tag specifying if the coordinates for a cell within a \ref
 * structure3D are relative to the virtual or real origin of the structure.
 */
enum coord_relativity {
  ekVORIGIN,
  ekRORIGIN
};

NS_END(structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_COORD_RELATIVITY_HPP_ */
