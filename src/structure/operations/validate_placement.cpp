/**
 * \file validate_placement.cpp
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
#include "silicon/structure/operations/validate_placement.hpp"

#include "cosm/repr/cube_block3D.hpp"
#include "cosm/repr/ramp_block3D.hpp"

#include "silicon/structure/structure3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
bool validate_placement::operator()(const crepr::cube_block3D* block) const {
  /* @todo check if structure invariants are violated by adding this block */
  return true;

error:
  ER_ERR("Cube block%d placement at %s,z_rot=%s failed validation",
         block->id().v(),
         mc_loc.to_str().c_str(),
         mc_z_rot.to_str().c_str());
  return false;
} /* operator() */

bool validate_placement::operator()(const crepr::ramp_block3D* block) const {
  if (rmath::radians::kZERO == mc_z_rot) {
    auto& extent_cell =
        mc_structure->access(mc_loc.x() + 1, mc_loc.y(), mc_loc.z());
    ER_CHECK(mc_structure->block_placement_cell_check(extent_cell),
             "Extent cell@%s failed validation for block placement",
             extent_cell.loc().to_str().c_str());
    /* @todo check if structure invariants are violated by adding this block */
    return true;
  } else {
    auto& extent_cell =
        mc_structure->access(mc_loc.x(), mc_loc.y() + 1, mc_loc.z());
    ER_CHECK(mc_structure->block_placement_cell_check(extent_cell),
             "Extent cell@%s failed validation for block placement",
             extent_cell.loc().to_str().c_str());
    /* @todo check if structure invariants are violated by adding this block */
    return true;
  }
error:
  ER_ERR("Ramp block%d placement at %s,z_rot=%s failed validation",
         block->id().v(),
         mc_loc.to_str().c_str(),
         mc_z_rot.to_str().c_str());
  return false;
} /* operator() */

NS_END(operations, structure, silicon);
