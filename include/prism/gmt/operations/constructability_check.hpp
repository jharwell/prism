/**
 * \file constructability_check.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * This file is part of PRISM.
 *
 * PRISM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * PRISM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * PRISM.  If not, see <http://www.gnu.org/licenses/
 */

#ifndef INCLUDE_PRISM_GMT_OPERATIONS_CONSTRUCTABILITY_CHECK_HPP_
#define INCLUDE_PRISM_GMT_OPERATIONS_CONSTRUCTABILITY_CHECK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/radians.hpp"

#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace prism::gmt::ds {
class connectivity_graph;
} /* namespace prism::gmt::ds */

namespace prism::gmt::repr {
class vshell;
} /* namespace prism::gmt::repr */

NS_START(prism, gmt, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class constructability_check
 * \ingroup gmt operations
 *
 * \brief Validate that the \ref connectivity_graph used to create a \ref spc_gmt is
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
      : ER_CLIENT_INIT("prism.gmt.operations.constructability_check") {}

  /* Not copy constructible or copy assignment by default  */
  constructability_check(const constructability_check&) = delete;
  constructability_check& operator=(const constructability_check&) = delete;

  bool operator()(const pgds::connectivity_graph* graph,
                  const pgrepr::vshell* vshell,
                  const rmath::radians& orientation) const;
};

NS_END(operations, gmt, prism);

#endif /* INCLUDE_PRISM_GMT_OPERATIONS_CONSTRUCTABILITY_CHECK_HPP_ */
