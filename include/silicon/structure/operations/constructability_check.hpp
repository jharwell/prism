/**
 * \file constructability_check.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_OPERATIONS_CONSTRUCTABILITY_CHECK_HPP_
#define INCLUDE_SILICON_STRUCTURE_OPERATIONS_CONSTRUCTABILITY_CHECK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/radians.hpp"

#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace silicon::structure::ds {
class spec_graph;
} /* namespace silicon::structure::ds */

namespace silicon::structure::repr {
class vshell;
} /* namespace silicon::structure::repr */

NS_START(silicon, structure, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class constructability_check
 * \ingroup structure operations
 *
 * \brief Validate that the \ref spec_graph used to create a \ref structure3D is
 * valid; that is, it satisfies the necessary properties in order to guarantee
 * construction by robots. This is a DIFFERENT operation that validating the
 * CURRENT state of a structure/whether or not a block placement at a given
 * location violates any of the properties (\ref block_placement_validate).
 *
 * The two operations may be merged once they mature, if it makes sense to do
 * so.
 *
 * A constructible specification is one that:
 *
 * - Satisfies \ref geometry_check.
 *
 * - Has all construction lanes be traversable by all robots, satisfying \ref
 *   traversability_check.
 *
 * - Has all structure layers be composable, satisfying \ref
 *   composability_check.
 *
 * - Does not violate any topological constraints, satisfying \ref
 *   topology_check.
 */
class constructability_check : public rer::client<constructability_check> {
 public:
  constructability_check(void)
      : ER_CLIENT_INIT("silicon.structure.operations.constructability_check") {}

  /* Not copy constructible or copy assignment by default  */
  constructability_check(const constructability_check&) = delete;
  constructability_check& operator=(const constructability_check&) = delete;

  bool operator()(const ssds::spec_graph* graph,
                  const ssrepr::vshell* vshell,
                  const rmath::radians& orientation) const;
};

NS_END(operations, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_OPERATIONS_CONSTRUCTABILITY_CHECK_HPP_ */
