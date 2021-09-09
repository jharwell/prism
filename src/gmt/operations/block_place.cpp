/**
 * \file block_place.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/gmt/operations/block_place.hpp"

#include "cosm/repr/cube_block3D.hpp"
#include "cosm/repr/ramp_block3D.hpp"

#include "prism/gmt/spc_gmt.hpp"
#include "prism/gmt/subtarget.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, operations);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
cpal::embodied_block_variantno
block_place::operator()(std::unique_ptr<cpal::embodied_cube_block> block) const {
  ER_INFO("Cube block%d to structure with intent: %s",
          block->id().v(),
          rcppsw::to_string(mc_intent).c_str());

  auto coord = mc_intent.site().to_real();
  /*
   * Set the new location for the block (equivalent to the absolute location of
   * the host cell in the arena).
   *
   * The discrete location does not match up to the real location, but ARGoS
   * does not use that, so that SHOULD be OK for now.
   */
  rmath::vector3d anchor_loc = m_structure->anchor_loc_abs(coord);
  block->ranchor3D(anchor_loc + embodiment_offset_calc(block.get()));
  block->danchor3D(m_structure->rorigind() + coord.offset());

  /* actually add the block to the structure */
  auto* ret = block.get();
  m_structure->block_add(std::move(block));

  /* update subtarget */
  auto* subtarget = m_structure->parent_subtarget(coord);
  subtarget->placed_count_update(1);
  return { ret };
} /* do_place() */

cpal::embodied_block_variantno
block_place::operator()(std::unique_ptr<cpal::embodied_ramp_block> block) const {
  ER_INFO("Cube block%d to structure with intent: %s",
          block->id().v(),
          rcppsw::to_string(mc_intent).c_str());

  auto coord = mc_intent.site().to_virtual();

  /*
   * Set the new location for the block (equivalent to the absolute location of
   * the host cell in the arena).
   *
   * The discrete location does not match up to the real location, but ARGoS
   * does not use that, so that SHOULD be OK for now.
   */
  rmath::vector3d anchor_loc = m_structure->anchor_loc_abs(coord);
  block->ranchor3D(anchor_loc + embodiment_offset_calc(block.get()));
  block->danchor3D(m_structure->vorigind() + coord.offset());

  /* actually add the block to the structure */
  auto* ret = block.get();
  m_structure->block_add(std::move(block));

  /* update subtarget */
  auto* subtarget = m_structure->parent_subtarget(coord);
  subtarget->placed_count_update(1);
  return ret;
} /* do_place() */

rmath::vector3d
block_place::embodiment_offset_calc(const crepr::base_block3D* block) const {
  /* X,Y axes of block and arena align */
  if (rmath::radians::kZERO == mc_intent.z_rot()) {
    return { block->rdims3D().x() / 2.0, block->rdims3D().y() / 2.0, 0.0 };
  } else { /* X,Y axes of block and arena are swapped */
    return { block->rdims3D().y() / 2.0, block->rdims3D().x() / 2.0, 0.0 };
  }
} /* embodiment_offset_calc() */

NS_END(operations, gmt, prism);
