/**
 * \file validate_spec_layers.cpp
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
#include "silicon/structure/operations/validate_spec_layers.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, operations);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool validate_spec_layers::operator()(void) const {
  for (size_t z = 0; z < mc_structure->zdsize(); ++z) {
    auto slice =
        slice2D(slice2D::coords_calc(rmath::vector3z::Z, mc_structure, z),
                mc_structure);
    ER_CHECK(layer_validate(slice, mc_structure->orientation(), z),
             "Layer%zu failed validation",
             z);
    ER_DEBUG("Layer%zu OK", z);
  } /* for(z..) */
  ER_INFO("All layers validated");
  return true;

error:
  return false;
} /* operator()() */

bool validate_spec_layers::layer_validate(const slice2D& layer,
                                          const rmath::radians& orientation,
                                          RCSW_UNUSED size_t z) const {
  /* PROPERTY: Hamiltonian AND even # rows along orientation direction */
  ER_CHECK(layer.is_hamiltonian(), "Layer%zu not Hamiltonian", z);
  if (rmath::radians::kZERO == mc_structure->orientation()) {
    ER_CHECK(RCSW_IS_EVEN(layer.d2()),
             "Structure oriented along X axis does not have even # rows in Y");
  } else if (rmath::radians::kPI_OVER_TWO == mc_structure->orientation()) {
    ER_CHECK(RCSW_IS_EVEN(layer.d1()),
             "Structure oriented along Y axis does not have even # rows in X");
  }
  /* PROPERTY: Must be traversable by robots in the specified orientation */
  ER_CHECK(layer.is_traversable(orientation),
           "Layer%zu not traversible by robots in orientation '%s'",
           z,
           rcppsw::to_string(orientation).c_str());

  /* PROPERTY: Must have Euler characteristic of 2 (not a torus) */

  return true;

error:
  return false;
} /* layer_validate() */

NS_END(operations, structure, silicon);
