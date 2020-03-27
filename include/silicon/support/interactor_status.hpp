/**
 * \file interactor_status.hpp
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

#ifndef INCLUDE_SILICON_SUPPORT_INTERACTOR_STATUS_HPP_
#define INCLUDE_SILICON_SUPPORT_INTERACTOR_STATUS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include "rcppsw/utils/maskable_enum.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, support);

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
   * \brief No event occured (i.e. the robot did not meet the requirements for
   * the interaction).
   */
  ekNO_EVENT = 1 << 0,

  /**
   * \brief The robot placed a block on the \ref structure3D.
   */
  ekSTRUCTURE_BLOCK_PLACE = 1 << 1,
};

NS_END(support, silicon);

#endif /* INCLUDE_SILICON_SUPPORT_INTERACTOR_STATUS_HPP_ */
