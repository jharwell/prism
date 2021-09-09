/**
 * \file block_spec.hpp
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

#ifndef INCLUDE_PRISM_GMT_REPR_BLOCK_SPEC_HPP_
#define INCLUDE_PRISM_GMT_REPR_BLOCK_SPEC_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/math/radians.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/ds/graph/hgrid3D_vertex_property.hpp"

#include "cosm/repr/block_type.hpp"
#include "cosm/repr/base_block3D.hpp"

#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/

NS_START(prism, gmt, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct block_anchor_spec
 * \ingroup gmt repr
 *
 * \brief Contains all the information need to fully specify the placement of a
 * given block at its anchor point on a \ref spc_gmt. These are the vertices in
 * the \ref connectivity_graph.
 */
struct block_anchor_spec : public rdgraph::hgrid3D_vertex_property {
  /**
   * \brief What type of block is it ?
   */
  crepr::block_type type{};

  /**
   * \brief What is the orientation of the block around its anchor?
   */
  rmath::radians z_rot{};

  /**
   * \brief The block at this anchor location, if any.
   */
  const crepr::base_block3D* block{nullptr};
};

/**
 * \struct block_extent_spec
 * \ingroup gmt repr
 *
 * \brief Contains all the information need to fully specify the extent of a
 * block in a given direction, given its \ref block_anchor_spec. These are
 * attached to the edges the \ref connectivity_graph.
 */
struct block_extent_spec {
  size_t weight{ 0 };
};

/**
 * \struct block_placement_spec
 * \ingroup gmt repr
 *
 * \brief The values in the \ref block_placement_map, which contain the placed
 * block and a pointer to the spec in the \ref spec_grap associated with the
 * block.
 */
struct block_placement_spec {
  const block_anchor_spec* spec{nullptr};
  std::unique_ptr<crepr::base_block3D> block{nullptr};
};

NS_END(repr, gmt, prism);

#endif /* INCLUDE_PRISM_GMT_REPR_BLOCK_SPEC_HPP_ */
