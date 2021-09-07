/**
 * \file composability_check.cpp
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
#include "silicon/structure/operations/composability_check.hpp"

#include "silicon/structure/operations/topology_check.hpp"
#include "silicon/structure/repr/vshell.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, operations);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
composability_check::composability_check(void)
    : ER_CLIENT_INIT("silicon.structure.operations.composability_check") {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool composability_check::operator()(const ssds::connectivity_graph* graph,
                                     const ssrepr::vshell* vshell) const {
  for (size_t z = 0; z < vshell->real()->zdsize() - 1; ++z) {
    auto lower = ssrepr::slice2D(ssrepr::slice2D::spec_calc(rmath::vector3z::Z,
                                                            z,
                                                            vshell),
                                 graph);
    auto upper = ssrepr::slice2D(ssrepr::slice2D::spec_calc(rmath::vector3z::Z,
                                                            z + 1,
                                                            vshell),
                                 graph);

    ER_CHECK(is_composable(lower, upper, vshell),
             "Layer%zu -> layer%zu not composable",
             z,
             z + 1);
    ER_DEBUG("Layer%zu -> layer%zu OK", z, z + 1);
  } /* for(z..) */
  return true;

error:
  return false;
} /* operator()() */

bool composability_check::is_composable(const ssrepr::slice2D& lower,
                                        const ssrepr::slice2D& upper,
                                        const ssrepr::vshell* vshell) const {
  auto topo_checker = topology_check();

  /*
   * \todo For right now, we disallow ALL holes in structures, because even
   * simple holes are not feasible unless beam blocks exist, which they do not
   * yet. This may be relaxed at some point in the future.
   */
  ER_CHECK(topo_checker.topological_holes(lower, vshell).empty() &&
           topo_checker.topological_holes(upper, vshell).empty(),
           "One or both layers contain topological holes");

  ER_CHECK(!has_overhangs(lower, upper), "Upper layer has overhangs");

  /*
   * \todo For now,  since only cube blocks are implemented, no more checking is
   * needed.
   */

  return true;

error:
  return false;
} /* is_composable() */

NS_END(operations, structure, silicon);
