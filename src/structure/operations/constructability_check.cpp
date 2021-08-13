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

#include "silicon/structure/operations/spec_composability_validate.hpp"
#include "silicon/structure/operations/geometry_check.hpp"
#include "silicon/structure/ds/spec_graph.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
bool constructability_check::operator()(const ssds::spec_graph* graph,
                                        const ssrepr::vshell* vshell,
                                        const rmath::radians& orientation) const {
  spec_composability_validate composable(graph);

  ER_CHECK(geometry_check()(graph, vshell, orientation),
           "Geometry validation failed");

  /* Second, inter-layer composability checks */
  ER_CHECK(composable(), "Structure layers are not composable");

  /* Third, topological checks:
   *
   * - No vertical topological holes (should be computable by the same algorithm
   *   that computes holes for sliced layers on the Z-axis); eg just slice along
   *   the Y axis. Maybe this should be part of composability checks?
   */
  return true;

error:
  return false;
} /* operator() */

NS_END(operations, structure, silicon);
