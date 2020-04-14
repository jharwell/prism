/**
 * \file validate_spec_layers.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_OPERATIONS_VALIDATE_SPEC_LAYERS_HPP_
#define INCLUDE_SILICON_STRUCTURE_OPERATIONS_VALIDATE_SPEC_LAYERS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <set>

#include "silicon/silicon.hpp"
#include "silicon/structure/structure3D.hpp"
#include "silicon/structure/slice2D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class validate_spec_layers
 * \ingroup structure operations
 *
 * \brief Perform intra-layer validation checks on the the specification for a
 * given layer as part of the specification validation process for an entire
 * \ref structure3D.
 */
class validate_spec_layers : public rer::client<validate_spec_layers> {
 public:
  explicit validate_spec_layers(const structure3D* structure)
      : ER_CLIENT_INIT("silicon.structure.operations.validate_spec_layers"),
        mc_structure(structure) {}

  /* Not copy constructable/assignable by default */
  validate_spec_layers(const validate_spec_layers&) = delete;
  const validate_spec_layers& operator=(const validate_spec_layers&) = delete;

  bool operator()(void) const;

 private:
  /**
   * \brief Validate the connectivity/structure of a given layer.
   *
   * This includes:
   *
   * - Not a torus (Euler characteristic is 2)
   * - Hamiltonian graph/even # of rows along orientation direction
   * - Traversible by robots in the specified orientation
   *
   * @todo Figure out more precise definitions for the properties that I am
   * validating from topology/graph theory.
   */
  bool layer_validate(const slice2D& layer,
                      const rmath::radians& orientation,
                      size_t z) const;
  /* clang-format off */
  const structure3D* mc_structure;
  /* clang-format on */
};

NS_END(operations, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_OPERATIONS_VALIDATE_SPEC_LAYERS_HPP_ */
