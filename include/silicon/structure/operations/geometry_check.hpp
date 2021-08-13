/**
 * \file geometry_check.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_OPERATIONS_GEOMETRY_CHECK_HPP_
#define INCLUDE_SILICON_STRUCTURE_OPERATIONS_GEOMETRY_CHECK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"

#include "silicon/structure/repr/slice2D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace silicon::structure::ds {
class spec_graph;
} /* namespace silicon::structure */

namespace silicon::structure::repr {
class vshell;
} /* namespace silicon::structure::repr */

NS_START(silicon, structure, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class geometry_check
 * \ingroup structure operations
 *
 * \brief Perform basic geometry checks as part of determining if a \ref
 * spec_graph corresponds to a constructible \ref structure3D.
 *
 * These include:
 *
 * - The orientation is valid.
 *
 * - Each layer's width is evenly divisible by the construction lane width in
 *   the orientation direction.
 */
class geometry_check : public rer::client<geometry_check> {
 public:
  geometry_check(void)
      : ER_CLIENT_INIT("silicon.structure.operations.geometry_check") {}


  /* Not copy constructable/assignable by default */
  geometry_check(const geometry_check&) = delete;
  const geometry_check& operator=(const geometry_check&) = delete;

  bool operator()(const ssds::spec_graph* spec,
                  const ssrepr::vshell* vshell,
                  const rmath::radians& orientation) const;

 private:
  /**
   * \brief Perform geometry checks on a single layer within the \ref
   * spec_graph.
   */
  bool layer_check(const ssrepr::slice2D& layer,
                   const rmath::radians& orientation) const;
};

NS_END(operations, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_OPERATIONS_GEOMETRY_CHECK_HPP_ */
