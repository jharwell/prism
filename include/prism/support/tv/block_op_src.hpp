/**
 * \file block_op_src.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, support, tv);

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
  ekARENA_PICKUP,

  /**
   * \brief Manipulate a block somewhere on an in-progress (construction target)
   * \ref spc_gmt.
   */
  ekCT_BLOCK_MANIP,
};

NS_END(tv, support, prism);

