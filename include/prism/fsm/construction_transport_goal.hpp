/**
 * \file construction_transport_goal.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, fsm);

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * \brief Represents the different locations/entities that a robot can transport
 * a block to during construction once they have picked one up.
 */
enum class construction_transport_goal {
  /**
   * \brief No goal--robot is probably not carrying a block.
   */
  ekNONE = -1,

  /**
   * A robot has acquired and picked up a block and is currently taking it to
   * the construction site.
   */
  ekCONSTRUCTION_SITE,

  /**
   * A robot has arrived in the construction site and is taking its block
   * somewhere on the construct target.
   */
  ekCT_BLOCK_PLACEMENT_SITE
};

NS_END(fsm, prism);

