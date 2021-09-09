/**
 * \file interactor_status.hpp
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

#ifndef INCLUDE_PRISM_SUPPORT_INTERACTOR_STATUS_HPP_
#define INCLUDE_PRISM_SUPPORT_INTERACTOR_STATUS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, support);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief The status returned by an action class representing a robot
 * interaction with the arena/environment. Used to disambiguate when
 * post-interaction hooks need to be run after processing all interactions for a
 * given robot on a given timestep.
 */
enum class interactor_status {
  /**
   * No event occured (i.e. the robot did not meet the requirements for the
   * interaction).
   */
  ekNO_EVENT = 1 << 0,

  /**
   * The robot placed a block on the \ref spc_gmt.
   */
  ekCT_BLOCK_PLACE = 1 << 1,

  /**
   * A robot picked up a block from the arena.
   */
  ekARENA_FREE_BLOCK_PICKUP = 1 << 2,

  /**
   * A robot stopped operation permanently (until the simulation is reset).
   */
  ekROBOT_STOPPED = 1 << 3
};

NS_END(support, prism);

#endif /* INCLUDE_PRISM_SUPPORT_INTERACTOR_STATUS_HPP_ */
