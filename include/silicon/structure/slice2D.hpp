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

#ifndef INCLUDE_SILICON_STRUCTURE_SLICE2D_HPP_
#define INCLUDE_SILICON_STRUCTURE_SLICE2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <set>

#include "rcppsw/ds/grid3D.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/ds/cell3D.hpp"

#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure);
class structure3D;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class slice2D
 * \ingroup structure
 *
 * \brief A 2D slice of a \ref structure3D along the X, Y, or Z axis.
 */
class slice2D : public rer::client<slice2D> {
 public:
  struct slice_coords {
    rmath::vector3z axis{};
    size_t offset{};
    rmath::vector3z ll{};
    rmath::vector3z ur{};
  };

  using layer_view = rds::grid3D<cds::cell3D>::const_grid_view;
  using topological_hole_type = std::set<const cds::cell3D*>;

  /**
   * \brief Given the axis for slicing, calculate the lower left and upper right
   * coordinates for the slice (defines the bounding box for the slice).
   */
  static slice_coords coords_calc(const rmath::vector3z& axis,
                                  const structure3D* structure,
                                  size_t offset);

  slice2D(const slice_coords& coords, const structure3D* structure);

  /* Not copy constructable/assignable by default */
  slice2D(const slice2D&) = delete;
  const slice2D& operator=(const slice2D&) = delete;

  /**
   * \brief Determine if the grid graph represented by the slice is Hamiltonian
   * (a Hamiltonian cycle exists)
   *
   * This check is valid regardless of contiguousness of the layer, because it
   * does not take traversability into account, and is mainly used to ensure
   * that creation of construction lanes later is valid.
   */
  bool is_hamiltonian(void) const;

  /**
   * \brief Determine if the slice is traversible in the specified direction by
   * robots. Should really only be called for slices that originated from the Z
   * axis...
   */
  bool is_traversable(const rmath::radians& orientation) const;

  /**
   * \brief Get the set of topological holes in the slice, where each
   * topological hole is a contiguous group of "simple" holes. Contiguousness is
   * determined by N,S,E,W adjacency.
   */
  std::set<topological_hole_type> topological_holes(void) const;

  const cds::cell3D& access(size_t d1, size_t d2) const;
  size_t d1(void) const;
  size_t d2(void) const;

  /**
   * \brief Return \c TRUE if the slice contains the specified coordinates, and
   * \c FALSE otherwise.
   */
  bool contains(const rmath::vector3z& coord) const;

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

  /**
   * \brief Determine if the layer is physically feasible for construction.
   *
   * - Ramp block extents are properly specified (no other blocks in them).
   */
  bool is_feasible(void) const;

  /**
   * \brief Get the set of "simple" holes in the slice; that is, cells which are
   * (1) empty in the structure final spec, and (2) not an exterior cell (not on
   * the edges of the slice).
   */
  std::set<const cds::cell3D*> simple_holes(void) const;

  /**
   * \brief Determine if a cell is a "simple" hole. Simple is defined by:
   *
   * - Not an exterior cell.
   * - Final cell state is EMPTY.
   */
  bool cell_is_simple_hole(const cds::cell3D& cell) const;

  bool cell_is_exterior(const cds::cell3D& cell) const;
  bool cells_are_adjacent(const cds::cell3D& cell1,
                          const cds::cell3D& cell2) const;

  /* clang-format off */
  const slice_coords mc_coords;
  const layer_view   mc_view;
  const structure3D* mc_structure;
  /* clang-format on */
};

NS_END(structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_SLICE2D_HPP_ */
