/**
 * \file place_block.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/structure/operations/place_block.hpp"

#include "cosm/repr/cube_block3D.hpp"
#include "cosm/repr/ramp_block3D.hpp"
#include "silicon/structure/operations/cell3D_block_extent.hpp"
#include "silicon/structure/operations/cell3D_block_place.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, operations);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void place_block::operator()(crepr::cube_block3D* block) const {
  /* update host cell */
  cell3D_block_place_visitor op(mc_loc, block);
  op.visit(*m_structure);

  /* actually add the block to the structure */
  m_structure->block_add(block);
}

void place_block::operator()(crepr::ramp_block3D* block) const {
  /* update host cell */
  cell3D_block_place_visitor host_op(mc_loc, block);
  host_op.visit(*m_structure);

  /* update cells for block extent */
  if (rmath::radians::kZERO == mc_z_rot) {
    for (size_t x = mc_loc.x(); x < mc_loc.x() + block->dims().x(); ++x) {
      rmath::vector3u extent_loc(x, mc_loc.y(), mc_loc.z());
      cell3D_block_extent_visitor op(extent_loc, block);
      op.visit(*m_structure);
    } /* for(x..) */
  } else if (rmath::radians::kPI_OVER_TWO == mc_z_rot) {
    for (size_t y = mc_loc.y(); y < mc_loc.y() + block->dims().y(); ++y) {
      rmath::vector3u extent_loc(mc_loc.x(), y, mc_loc.z());
      cell3D_block_extent_visitor op(extent_loc, block);
      op.visit(*m_structure);
    } /* for(y..) */
  } else {
    ER_FATAL_SENTINEL("Bad Z rotation %s for block%d specified",
                      mc_z_rot.to_str().c_str(),
                      block->id().v());
  }

  /* actually add the block to the structure */
  m_structure->block_add(block);
}

NS_END(operations, structure, silicon);
