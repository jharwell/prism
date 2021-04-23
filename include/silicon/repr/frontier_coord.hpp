/**
 * \file frontier_coord.hpp
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

#ifndef INCLUDE_SILICON_REPR_FRONTIER_COORD_HPP_
#define INCLUDE_SILICON_REPR_FRONTIER_COORD_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/silicon.hpp"
#include "silicon/structure/ds/ct_coord.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief Representation of how far into a construction lane construction has
 * progressed so far.
 */

struct frontier_coord {
  ssds::ct_coord ingress;
  ssds::ct_coord egress;
};

NS_END(repr, silicon);

#endif /* INCLUDE_SILICON_REPR_FRONTIER_COORD_HPP_ */
