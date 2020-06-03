/**
 * \file validate_spec_composability.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_OPERATIONS_VALIDATE_SPEC_COMPOSABILITY_HPP_
#define INCLUDE_SILICON_STRUCTURE_OPERATIONS_VALIDATE_SPEC_COMPOSABILITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant/static_visitor.hpp>

#include "rcppsw/er/client.hpp"

#include "silicon/silicon.hpp"
#include "silicon/structure/slice2D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure);

class structure3D;

NS_START(operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class validate_spec_composability
 * \ingroup structure operations
 *
 * \brief Determine if the layers of the \ref structure3D are composable; that
 * is, they can be stacked on top of each other without violating any
 * graphical/topological invariants.
 */
class validate_spec_composability : public rer::client<validate_spec_composability>,
                      public boost::static_visitor<bool> {
 public:
  explicit validate_spec_composability(const structure3D* structure)
      : ER_CLIENT_INIT("silicon.structure.operations.validate_spec_composability"),
        mc_structure(structure) {}

  /* Not copy constructible or copy assignment by default  */
  validate_spec_composability(const validate_spec_composability&) = delete;
  validate_spec_composability& operator=(const validate_spec_composability&) = delete;

  bool operator()(void) const;

 private:
  /**
   * \brief Determine if the \p lower and \p upper layers at z, z+1, are
   * composable that is, can be stacked/combined to produce a joint graph which
   * is:
   *
   * - Physically feasible
   */
  bool is_composable(const slice2D& lower, const slice2D& upper) const;

  /* clang-format off */
  const structure3D* mc_structure;
  /* clang-format on */
};

NS_END(operations, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_OPERATIONS_VALIDATE_SPEC_COMPOSABILITY_HPP_ */
