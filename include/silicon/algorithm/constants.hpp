/**
 * \file constants.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_SILICON_ALGORITHM_CONSTANTS_HPP_
#define INCLUDE_SILICON_ALGORITHM_CONSTANTS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, algorithm, constants);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief The # of cells that robots need to keep between them and the robot in
 * front of them, once they have acquired the frontier set, in order to ensure
 * deadlock-free operation.
 */
static constexpr size_t kCT_FRONTIER_SET_PROX_CELLS = 3;

/**
 * \brief The # of cells that robots need to keep between them and the robot in
 * front of them when they are in a construction lane (ingress/egress), but have
 * not yet acquired the frontier set.
 */
static constexpr size_t kCT_TRAJECTORY_PROX_CELLS = 2;

/**
 * \brief The width of a construction lane/subtarget on the structure in cells.
 */
static constexpr const size_t kCT_SUBTARGET_WIDTH_CELLS = 2;


NS_END(constants, algorithm, silicon);

#endif /* INCLUDE_SILICON_ALGORITHM_CONSTANTS_HPP_ */
