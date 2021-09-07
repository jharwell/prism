/**
 * \file constructability_check.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/structure/operations/constructability_check.hpp"

#include "silicon/structure/operations/composability_check.hpp"
#include "silicon/structure/operations/geometry_check.hpp"
#include "silicon/structure/operations/traversability_check.hpp"
#include "silicon/structure/operations/topology_check.hpp"
#include "silicon/structure/ds/connectivity_graph.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
bool constructability_check::operator()(const ssds::connectivity_graph* graph,
                                        const ssrepr::vshell* vshell,
                                        const rmath::radians& orientation) const {
  ER_CHECK(geometry_check()(graph, vshell, orientation),
           "Structure spec failed Geometrical validation");

  ER_CHECK(composability_check()(graph, vshell),
           "Structure spec not composable");

  ER_CHECK(traversability_check()(graph, vshell),
           "Structure spec not traversable at all points");

  ER_CHECK(topology_check()(graph, vshell),
           "Structure spec violates topological invariants")

  return true;

error:
  return false;
} /* operator() */

NS_END(operations, structure, silicon);
