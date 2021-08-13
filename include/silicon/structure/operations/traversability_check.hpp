/**
 * \file traversability_check.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_OPERATIONS_TRAVERSABILITY_CHECK_HPP_
#define INCLUDE_SILICON_STRUCTURE_OPERATIONS_TRAVERSABILITY_CHECK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/silicon.hpp"
#include "silicon/structure/repr/slice2D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace silicon::structure::ds {
class spec_graph;
} /* namespace silicon::structure::ds */

NS_START(silicon, structure, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class traversability_check
 * \ingroup structure operations
 *
 * \brief Perform traversability checks as part of determining if a \ref
 * spec_graph corresponds to a constructible \ref structure3D by checking if
 * every \ref slice2D along the Z axis is traversible by a robot.
 */
class traversability_check : public rer::client<traversability_check> {
 public:
  traversability_check(void)
      : ER_CLIENT_INIT("silicon.structure.operations.traversability_check") {}

  /* Not move/copy constructable/assignable by default */
  traversability_check(const traversability_check&) = delete;
  traversability_check& operator=(const traversability_check&) = delete;
  traversability_check(traversability_check&&) = delete;
  traversability_check& operator=(traversability_check&&) = delete;

  bool operator()(const ssds::spec_graph* spec,
                  const ssrepr::vshell* vshell,
                  const rmath::radians& orientation) const;

 private:
  /**
   * \brief Determine if the layer is traversable by robots entering/exiting
   * from the first slice dimension in the negative direction for all values.
   */
  bool is_traversable_d1_neg(void) const;

  /**
   * \brief Determine if the layer is traversable by robots entering/exiting
   * from the second slice dimension in the negative direction for all values.
   */
  bool is_traversable_d2_neg(void) const;

  /**
   * \brief Determine if the layer is traversable by robots entering/exiting
   * from the first slice dimension in the positive direction for all values.
   */
  bool is_traversable_d1_pos(void) const;

  /**
   * \brief Determine if the layer is traversable by robots entering/exiting
   * from the second slice dimension in the positive direction for all values.
   */
  bool is_traversable_d2_pos(void) const;
};

NS_END(operations, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_OPERATIONS_TRAVERSABILITY_CHECK_HPP_ */
