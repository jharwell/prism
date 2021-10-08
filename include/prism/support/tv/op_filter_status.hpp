/**
 * \file op_filter_status.hpp
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
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, support, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief Contains the various statuses relating to robots and block operations
 * (picking up, dropping).
 */
enum class op_filter_status {
  /**
   * \brief The robot has passed all necessary filter checkes for the requested
   * operation.
   *
   */
  ekSATISFIED,

  /**
   * \brief The robot has not currently achieved the necessary internal state
   * for the block operation.
   */
  ekROBOT_INTERNAL_UNREADY,

  /**
   * \brief The robot has achieved the necessary internal state for the block
   * operation (probably a pickup), but is not actually on a block, so the
   * desired operation is invalid.
   */
  ekROBOT_NOT_ON_BLOCK
};

NS_END(tv, support, prism);

