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
#include "silicon/structure/subtarget.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, operations);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void place_block::operator()(crepr::cube_block3D* block) const {
  do_place(std::unique_ptr<crepr::cube_block3D>(block));
} /* operator()() */

void place_block::operator()(crepr::ramp_block3D* block) const {
  do_place(std::unique_ptr<crepr::ramp_block3D>(block));
} /* operator()()*/

void place_block::do_place(std::unique_ptr<crepr::cube_block3D> block) const {
  ER_INFO("Cube block%d to structure cell@%s,z_rot=%s",
          block->id().v(),
          mc_cell.to_str().c_str(),
          mc_z_rot.to_str().c_str());

  /* update host cell */
  cell3D_block_place_visitor op(mc_cell, block.get());
  op.visit(m_structure->access(mc_cell));

  /*
   * Set the new location for the block (equivalent to the absolute location of
   * the host cell in the arena).
   *
   * The discrete location does not match up to the real location, but ARGoS
   * does not use that, so that SHOULD be OK for now.
   */
  rmath::vector3d cell_loc =
      m_structure->cell_loc_abs(m_structure->access(mc_cell));
  block->rpos3D(cell_loc + embodiment_offset_calc(block.get()));
  block->dpos3D(m_structure->origind() + mc_cell);

  /* actually add the block to the structure */
  m_structure->block_add(std::move(block));

  /* update subtarget */
  auto subtarget = m_structure->cell_subtarget(m_structure->access(mc_cell));
  subtarget->placed_block_count_update(1);
} /* do_place() */

void place_block::do_place(std::unique_ptr<crepr::ramp_block3D> block) const {
  ER_INFO("Ramp block%d to structure cell@%s,z_rot=%s",
          block->id().v(),
          mc_cell.to_str().c_str(),
          mc_z_rot.to_str().c_str());
  /* update host cell */
  cell3D_block_place_visitor host_op(mc_cell, block.get());
  host_op.visit(m_structure->access(mc_cell));

  /*
   * Set the new location for the block (equivalent to the absolute location of
   * the host cell in the arena).
   *
   * The discrete location does not match up to the real location, but ARGoS
   * does not use that, so that SHOULD be OK for now.
   */
  rmath::vector3d cell_loc =
      m_structure->cell_loc_abs(m_structure->access(mc_cell));
  block->rpos3D(cell_loc + embodiment_offset_calc(block.get()));
  block->dpos3D(m_structure->origind() + mc_cell);

  /* update cells for block extent */
  if (rmath::radians::kZERO == mc_z_rot) {
    auto ub = static_cast<size_t>(mc_cell.x() +
                                    block->dims3D().x() / block->dims3D().y());
    for (size_t x = mc_cell.x() + 1; x < ub; ++x) {
      rmath::vector3z extent_loc(x, mc_cell.y(), mc_cell.z());
      cell3D_block_extent_visitor op(extent_loc, block.get());
      op.visit(m_structure->access(extent_loc));
    } /* for(x..) */
  } else if (rmath::radians::kPI_OVER_TWO == mc_z_rot) {
    auto ub = static_cast<size_t>(mc_cell.y() +
                                    block->dims3D().x() / block->dims3D().y());
    for (size_t y = mc_cell.y() + 1; y < ub; ++y) {
      rmath::vector3z extent_loc(mc_cell.x(), y, mc_cell.z());
      cell3D_block_extent_visitor op(extent_loc, block.get());
      op.visit(m_structure->access(extent_loc));
    } /* for(y..) */
  } else {
    ER_FATAL_SENTINEL("Bad Z rotation %s for block%d specified",
                      mc_z_rot.to_str().c_str(),
                      block->id().v());
  }

  /* actually add the block to the structure */
  m_structure->block_add(std::move(block));

  /* update subtarget */
  auto subtarget = m_structure->cell_subtarget(m_structure->access(mc_cell));
  subtarget->placed_block_count_update(1);
} /* do_place() */

rmath::vector3d place_block::embodiment_offset_calc(
    const crepr::base_block3D* block) const {
  /* X,Y axes of block and arena align */
  if (rmath::radians::kZERO == mc_z_rot) {
    return {block->dims3D().x() / 2.0, block->dims3D().y() / 2.0, 0.0};
  } else { /* X,Y axes of block and arena are swapped */
    return {block->dims3D().y() / 2.0, block->dims3D().x() / 2.0, 0.0};
  }
} /* embodiment_offset_calc() */

NS_END(operations, structure, silicon);
