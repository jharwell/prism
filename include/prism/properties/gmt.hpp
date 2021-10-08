/**
 * \file gmt.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include <string>

#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, properties, gmt);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief Cube blocks are restricted to being connected to their neighbors only
 * through their faces.
 */
static constexpr const size_t kMAX_MANHATTAN_NEIGHBORS_CUBE = 6;

/**
 * \brief Cube blocks are restricted to being connected to their neighbors only
 * through their faces. 1 for back + 2 for bottom + 2 for sides = 5 (sides would
 * have to be other ramp blocks).
 */
static constexpr const size_t kMAX_MANHATTAN_NEIGHBORS_RAMP = 5;

/**
 * \brief If a vertex on the structure has at least 1 neighbor above it (i.e.,
 * the vertex at (x,y,z+1) exists), then it must have at least 2 neighbors: its
 * support below, and the neighbor above. In most cases it will have more, but
 * this is as restrictive as I can be.
 */
static constexpr const size_t kABOVE_GROUND_MIN_MANHATTAN_NEIGHBORS = 2;

/**
 * \brief The color of cube blocks, for the purposes of doing structure
 * validation through graph coloring.
 */
static inline const std::string kVertexColorCube = "blue";

/**
 * \brief The color of ramp blocks, for the purposes of doing structure
 * validation through graph coloring.
 */
static inline const std::string kVertexColorRamp = "red";

NS_END(gmt, properties, prism);
