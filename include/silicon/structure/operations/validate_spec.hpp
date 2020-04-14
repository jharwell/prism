/**
 * \file validate_spec.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_OPERATIONS_VALIDATE_SPEC_HPP_
#define INCLUDE_SILICON_STRUCTURE_OPERATIONS_VALIDATE_SPEC_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant/static_visitor.hpp>

#include "rcppsw/er/client.hpp"

#include "silicon/silicon.hpp"
#include "silicon/structure/structure3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class validate_spec
 * \ingroup structure operations
 *
 * \brief Validate that the spec used to create a \ref structure3D is valid;
 * that is, it satisfies the necessary properties in order to guarantee
 * construction by robots. This is a DIFFERENT operation that validating the
 * CURRENT state of a structure/whether or not a block placement at a given
 * location violates any of the properties (\ref validate_placement)
 *
 * The two operations may be merged once they mature, if it makes sense to do
 * so.
 *
 * This operation is done AFTER the \ref structure3D object is created, because
 * the implementation details of the validation operation are MUCH easier on the
 * structure as opposed to lists/maps of block locations.
 */
class validate_spec : public rer::client<validate_spec>,
                      public boost::static_visitor<bool> {
 public:
  explicit validate_spec(const structure3D* structure)
      : ER_CLIENT_INIT("silicon.structure.operations.validate_spec"),
        mc_structure(structure) {}

  /* Not copy constructible or copy assignment by default  */
  validate_spec(const validate_spec&) = delete;
  validate_spec& operator=(const validate_spec&) = delete;

  bool operator()(void) const;

 private:
  struct n_vertices_ret_type {
    size_t cube;
    size_t ramp;
  };

  /* clang-format off */
  const structure3D* mc_structure;
  /* clang-format on */
};

NS_END(operations, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_OPERATIONS_VALIDATE_SPEC_HPP_ */
