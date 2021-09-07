/**
 * \file slice2D.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_REPR_SLICE2D_HPP_
#define INCLUDE_SILICON_STRUCTURE_REPR_SLICE2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector3.hpp"

#include "cosm/repr/graph3D_view_entity.hpp"

#include "silicon/structure/ds/connectivity_graph.hpp"
#include "silicon/structure/repr/slice2D_spec.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, repr);
class vshell;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class slice2D
 * \ingroup structure repr
 *
 * \brief A 2D slice of a \ref structure3D along the X, Y, or Z axis.
 */
class slice2D : public rer::client<slice2D>,
                public crepr::graph3D_view_entity<typename ssds::connectivity_graph::decoratee_type,
                                                  typename ssds::connectivity_graph::view_type> {
 public:
  /**
   * \brief Given the axis for slicing, calculate the necessary info to create a
   * 2D slice (bounding box of vertices) of a \ref ssds::connectivity_graph.
   */
  static slice2D_spec spec_calc(const rmath::vector3z& axis,
                                size_t axis_offset,
                                const ssrepr::vshell* vshell);

  slice2D(const slice2D_spec& spec, const ssds::connectivity_graph* graph);

  /* Not copy constructable/assignable by default */
  slice2D(const slice2D&) = delete;
  const slice2D& operator=(const slice2D&) = delete;

  bool cells_are_adjacent(const rmath::vector3z& cell1,
                          const rmath::vector3z& cell2) const;

 private:
  /* clang-format off */
  const slice2D_spec mc_spec;
  /* clang-format on */
};

NS_END(repr, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_REPR_SLICE2D_HPP_ */
