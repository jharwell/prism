/**
 * \file static_build_status.hpp
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

#ifndef INCLUDE_PRISM_GMT_STATIC_BUILD_STATUS_HPP_
#define INCLUDE_PRISM_GMT_STATIC_BUILD_STATUS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief The status values return from the static build process by a \ref
 * spc_gmt_builder instance.
 */
enum class static_build_status {
  /**
   * \brief The structure is finished, so no work was done.
   */
  ekFINISHED,

  /**
   * \brief The next interval for building has not been hit yet, so nothing was
   * done.
   */
  ekNO_INTERVAL,

  /**
   * \brief The interval limit of # of blocks to build was hit, and all blocks
   * were placed successfully.
   */
  ekINTERVAL_LIMIT,

  /**
   * \brief One or more blocks failed to built during the static build process
   * during the interval.
   */
  ekINTERVAL_FAILURE
};

NS_END(gmt, prism);

#endif /* INCLUDE_PRISM_GMT_STATIC_BUILD_STATUS_HPP_ */
