/**
 * \file topology_check.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_PRISM_GMT_OPERATIONS_TOPOLOGY_CHECK_HPP_
#define INCLUDE_PRISM_GMT_OPERATIONS_TOPOLOGY_CHECK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <set>
#include <vector>

#include "rcppsw/math/vector3.hpp"

#include "prism/gmt/repr/block_spec.hpp"
#include "prism/gmt/repr/slice2D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace prism::gmt::ds {
class connectivity_graph;
} /* namespace prism::gmt */

namespace prism::gmt::repr {
class vshell;
} /* namespace prism::gmt::repr */

NS_START(prism, gmt, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class topology_check
 * \ingroup gmt operations
 *
 * \brief Perform topological checks as part of determining if a \ref connectivity_graph
 * corresponds to a constructible \ref spc_gmt.
 *
 * These include:
 *
 * - The graph contains no holes in any layer other than the top (required for
 *   structures composed of only cube blocks; will be relaxed later.
 */
class topology_check : public rer::client<topology_check> {
 public:
  using extent_hole = const pgrepr::block_anchor_spec*;
  using topological_hole = std::set<extent_hole>;

  topology_check(void);

  /* Not move/copy constructable/assignable by default */
  topology_check(const topology_check&) = delete;
  topology_check& operator=(const topology_check&) = delete;
  topology_check(topology_check&&) = delete;
  topology_check& operator=(topology_check&&) = delete;

  /**
   * \brief Perform topological checks on a all layers (siliced in X,Y,Z) within
   * the \ref connectivity_graph.
   */
  bool operator()(const pgds::connectivity_graph* graph,
                  const pgrepr::vshell* vshell) const;

  /**
   * \brief Perform topological checks on a single layer within the \ref
   * connectivity_graph.
   */
  bool layer_check(const pgrepr::slice2D& layer,
                   const pgrepr::vshell* vshell) const;

  /**
   * \brief Get the set of topological holes in the slice (i.e., within a single
   * layer), where each topological hole is a contiguous group of "simple"
   * holes. Contiguousness is determined by N,S,E,W adjacency.
   */
  std::vector<topological_hole> topological_holes(const pgrepr::slice2D& layer,
                                                  const pgrepr::vshell* vshell) const;

 private:


  /**
   * \brief Find all extent holes in a layer. Cells (x,y,z) in the bounding box
   * of the layer are considered extent holes if they meet any of the following
   * criteria.
   *
   * 1. Are not anchor holes.
   * 2. Are not crossed by exactly 1 edge of a block's extent (i.e., one of the
   *    neighboring blocks has an extent that is too short or too long). This is
   *    equivalent to saying L1 norm of the difference in coordinates of the two
   *    corresponding vertices (e = (u,v)) is not equal to the edge length.
   * 3. Are not on the exterior of the bounding box/a perimeter cell.
   *
   * \todo This check is only currently valid for structures containing only
   * cube and beam blocks.
   */
  std::set<extent_hole> extent_holes(const pgrepr::slice2D& layer,
                                     const pgrepr::vshell* vshell) const;

  /**
   * \brief Determine if the host cell of a block is an anchor hole: a cell
   * (x,y,z) in the bounding box of the spec which meets all of the following
   * criteria.
   *
   * 1. Contains no anchor point.
   * 2. Is not on the exterior of the bounding box/a perimeter cell.
   *
   * \todo This check is only currently valid for structures containing only
   * cube and beam blocks.
   */
  bool is_anchor_hole(const pgrepr::block_anchor_spec* spec,
                      const pgrepr::vshell* vshell) const;

  /**
   * \brief Determine if an anchor point is on the exterior of the structure
   * (i.e., has at least one coordinate = 0 or = the max value for that
   * dimension -1).
   */
  bool cell_on_exterior(const rmath::vector3z& cell,
                        const pgrepr::vshell* vshell) const;


  /* clang-format off */
  /* clang-format on */
};

NS_END(operations, gmt, prism);

#endif /* INCLUDE_PRISM_GMT_OPERATIONS_TOPOLOGY_CHECK_HPP_ */
