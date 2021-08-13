/**
 * \file geometry_check.cpp
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
#include "silicon/structure/operations/geometry_check.hpp"

#include "silicon/structure/repr/vshell.hpp"
#include "silicon/algorithm/constants.hpp"
#include "silicon/structure/utils.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, operations);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool geometry_check::operator()(const ssds::spec_graph* spec,
                                const ssrepr::vshell* vshell,
                                const rmath::radians& orientation) const {
  ER_CHECK(sstructure::orientation_valid(orientation),
           "Bad orientation: '%s'",
           rcppsw::to_string(orientation).c_str());

  for (size_t z = 0; z < vshell->real()->zdsize(); ++z) {
    auto slice = ssrepr::slice2D(ssrepr::slice2D::spec_calc(rmath::vector3z::Z,
                                                            z,
                                                            spec,
                                                            vshell),
                                 spec);
    ER_CHECK(layer_check(slice, orientation, z),
             "Layer%zu failed validation",
             z);
    ER_DEBUG("Layer%zu OK", z);
  } /* for(z..) */
  ER_INFO("All layers validated");
  return true;

error:
  return false;
} /* operator()() */

bool geometry_check::layer_check(const ssrepr::slice2D& layer,
                                 const rmath::radians& orientation) const {
  /*
   * PROPERTY: Layer has can be evenly divided into N construction lanes along
   * orientation direction.
   */
  if (rmath::radians::kZERO == orientation ||
      rmath::radians::kPI == orientation) {
    ER_CHECK(0 == (layer.d2() % saconstants::kCT_SUBTARGET_WIDTH_CELLS),
             "Spec with orientation={0,PI} not evenly divisible by subtarget width %zu",
             saconstants::kCT_SUBTARGET_WIDTH_CELLS);
  } else if (rmath::radians::kPI_OVER_TWO == orientation ||
             rmath::radians::kTHREE_PI_OVER_TWO == orientation) {
    ER_CHECK(0 == (layer.d1() % saconstants::kCT_SUBTARGET_WIDTH_CELLS),
             "Spec with orientation={PI/2,3PI/2} not evenly divisible by subtarget width %zu",
             saconstants::kCT_SUBTARGET_WIDTH_CELLS);
  }

  return true;

error:
  return false;
} /* layer_check() */

NS_END(operations, structure, silicon);
