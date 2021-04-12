/**
 * \file block_place.cpp
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
#include "silicon/structure/operations/block_place.hpp"

#include "cosm/repr/cube_block3D.hpp"
#include "cosm/repr/ramp_block3D.hpp"

#include "silicon/structure/operations/cell3D_block_extent.hpp"
#include "silicon/structure/operations/cell3D_block_place.hpp"
#include "silicon/structure/structure3D.hpp"
#include "silicon/structure/subtarget.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, operations);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
cpal::embodied_block_variantno
block_place::operator()(std::unique_ptr<cpal::embodied_cube_block> block) const {
  ER_INFO("Cube block%d to structure cell@%s,z_rot=%s",
          block->id().v(),
          rcppsw::to_string(mc_coord).c_str(),
          rcppsw::to_string(mc_z_rot).c_str());

  /* update host cell */
  cell3D_block_place_visitor op(mc_coord, block.get());
  op.visit(m_structure->access(mc_coord));

  /*
   * Set the new location for the block (equivalent to the absolute location of
   * the host cell in the arena).
   *
   * The discrete location does not match up to the real location, but ARGoS
   * does not use that, so that SHOULD be OK for now.
   */
  rmath::vector3d cell_loc =
      m_structure->cell_loc_abs(m_structure->access(mc_coord));
  block->ranchor3D(cell_loc + embodiment_offset_calc(block.get()));
  block->danchor3D(m_structure->vorigind() + mc_coord);

  /* actually add the block to the structure */
  auto ret = block.get();
  m_structure->block_add(std::move(block));

  /* update subtarget */
  auto subtarget =
      m_structure->parent_subtarget({ mc_coord, coord_relativity::ekVORIGIN });
  subtarget->placed_count_update(1);
  return { ret };
} /* do_place() */

cpal::embodied_block_variantno
block_place::operator()(std::unique_ptr<cpal::embodied_ramp_block> block) const {
  ER_INFO("Ramp block%d to structure cell@%s,z_rot=%s",
          block->id().v(),
          rcppsw::to_string(mc_coord).c_str(),
          rcppsw::to_string(mc_z_rot).c_str());
  /* update host cell */
  cell3D_block_place_visitor host_op(mc_coord, block.get());
  host_op.visit(m_structure->access(mc_coord));

  /*
   * Set the new location for the block (equivalent to the absolute location of
   * the host cell in the arena).
   *
   * The discrete location does not match up to the real location, but ARGoS
   * does not use that, so that SHOULD be OK for now.
   */
  rmath::vector3d cell_loc =
      m_structure->cell_loc_abs(m_structure->access(mc_coord));
  block->ranchor3D(cell_loc + embodiment_offset_calc(block.get()));
  block->danchor3D(m_structure->vorigind() + mc_coord);

  /* update cells for block extent */
  if (rmath::radians::kZERO == mc_z_rot) {
    auto ub = static_cast<size_t>(mc_coord.x() +
                                  block->rdim3D().x() / block->rdim3D().y());
    for (size_t x = mc_coord.x() + 1; x < ub; ++x) {
      rmath::vector3z extent_loc(x, mc_coord.y(), mc_coord.z());
      cell3D_block_extent_visitor op(extent_loc, block.get());
      op.visit(m_structure->access(extent_loc));
    } /* for(x..) */
  } else if (rmath::radians::kPI_OVER_TWO == mc_z_rot) {
    auto ub = static_cast<size_t>(mc_coord.y() +
                                  block->rdim3D().x() / block->rdim3D().y());
    for (size_t y = mc_coord.y() + 1; y < ub; ++y) {
      rmath::vector3z extent_loc(mc_coord.x(), y, mc_coord.z());
      cell3D_block_extent_visitor op(extent_loc, block.get());
      op.visit(m_structure->access(extent_loc));
    } /* for(y..) */
  } else {
    ER_FATAL_SENTINEL("Bad Z rotation %s for block%d specified",
                      rcppsw::to_string(mc_z_rot).c_str(),
                      block->id().v());
  }

  /* actually add the block to the structure */
  auto ret = block.get();
  m_structure->block_add(std::move(block));

  /* update subtarget */
  auto subtarget =
      m_structure->parent_subtarget({ mc_coord, coord_relativity::ekVORIGIN });
  subtarget->placed_count_update(1);
  return ret;
} /* do_place() */

rmath::vector3d
block_place::embodiment_offset_calc(const crepr::base_block3D* block) const {
  /* X,Y axes of block and arena align */
  if (rmath::radians::kZERO == mc_z_rot) {
    return { block->rdim3D().x() / 2.0, block->rdim3D().y() / 2.0, 0.0 };
  } else { /* X,Y axes of block and arena are swapped */
    return { block->rdim3D().y() / 2.0, block->rdim3D().x() / 2.0, 0.0 };
  }
} /* embodiment_offset_calc() */

NS_END(operations, structure, silicon);
