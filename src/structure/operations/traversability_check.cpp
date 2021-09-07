/**
 * \file traversability_check.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/structure/operations/traversability_check.hpp"

#include "silicon/structure/repr/vshell.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, operations);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool traversability_check::operator()(const ssds::connectivity_graph* spec,
                                      const ssrepr::vshell* vshell) const {
  /*
   * PROPERTY: Bottom layer (z=0) is ALWAYS traversable
   *
   * PROPERTY: Don't need to check the top layer, because robots never traverse
   * it.
   */
  for (size_t z = 1; z < vshell->real()->zdsize(); ++z) {
    ER_TRACE("Check traversability of layer%zu", z);

    /*
     * \todo For now,  since only cube blocks are implemented, all layers are
     * traversible at all points.
     */
    auto slice = ssrepr::slice2D(ssrepr::slice2D::spec_calc(rmath::vector3z::Z,
                                                            z,
                                                            vshell),
                                 spec);

    ER_DEBUG("Layer%zu OK", z);
  } /* for(z..) */

  return true;

error:
  return false;
} /* operator()() */


NS_END(operations, structure, silicon);
