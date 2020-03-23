/**
 * \file cell3D_target_state.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_CELL3D_TARGET_STATE_HPP_
#define INCLUDE_SILICON_STRUCTURE_CELL3D_TARGET_STATE_HPP_

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
enum class cell3D_target_state {
  ekEMPTY,
  ekCUBE_BLOCK,
  ekRAMP_BLOCK,
  ekBLOCK_EXTENT
};

NS_END(structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_CELL3D_TARGET_STATE_HPP_ */
