/**
 * \file block_op_src.hpp
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

#ifndef INCLUDE_SILICON_SUPPORT_TV_BLOCK_OP_SRC_HPP_
#define INCLUDE_SILICON_SUPPORT_TV_BLOCK_OP_SRC_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, support, tv);

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * \brief The different types of operations that can be performed on blocks.
 */
enum class block_op_src {
  /**
   * \brief Pick up a free block from somewhere in the the arena.
   */
  ekFREE_PICKUP,

  /**
   * \brief Place a block somewhere on the in-progress \ref structure3D.
   */
  ekSTRUCTURE_PLACEMENT,
};

NS_END(tv, support, silicon);

#endif /* INCLUDE_SILICON_SUPPORT_TV_BLOCK_OP_SRC_HPP_ */
