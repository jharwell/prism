/**
 * \file structure3D.cpp
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
#include "silicon/structure/structure3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
structure3D::structure3D(const config::structure3D_config* config)
    : grid3D(config->bounding_box),
      ER_CLIENT_INIT("silicon.structure.structure3D"),
      mc_anchor(config->anchor),
      mc_bounding_box(config->bounding_box),
      mc_orientation(config->orientation) {
    for (uint i = 0; i < mc_bounding_box.x(); ++i) {
      for (uint j = 0; j < mc_bounding_box.y(); ++j) {
        for (uint k = 0; k < mc_bounding_box.z(); ++k) {
          rmath::vector3u c(i, j, k);
          access(c).loc(c);
        } /* for(k..) */
      } /* for(j..) */
    } /* for(i..) */
  }

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool structure3D::block_addition_valid(const rmath::vector3u& loc,
                                       placement_orientation orientation,
                                       const crepr::base_block3D* block) {
  auto& cell = access(loc);
  ER_CHECK(block_addition_cell_check(cell),
           "Host Cell@%s failed validation for block addition",
           cell.loc().to_str().c_str());

  if (crepr::block_type::ekCUBE == block->md()->type()) {
    /* @todo check if structure invariants are violated by adding this block */
    return true;
  }

  /* harder case: ramp blocks */
  if (ekX == orientation) {
    auto& extent_cell = access(loc.x() + 1, loc.y(), loc.z());
    ER_CHECK(block_addition_cell_check(extent_cell),
             "Extent cell@%s failed validation for block addition",
             extent_cell.loc().to_str().c_str());
    /* @todo check if structure invariants are violated by adding this block */
    return true;
  } else {
    auto& extent_cell = access(loc.x(), loc.y() + 1, loc.z());
    ER_CHECK(block_addition_cell_check(extent_cell),
             "Extent cell@%s failed validation for block addition",
             extent_cell.loc().to_str().c_str());
    /* @todo check if structure invariants are violated by adding this block */
    return true;
  }

error:
  return false;
} /* block_addition_valid() */

bool structure3D::block_addition_cell_check(const cds::cell3D& cell) const {
  ER_CHECK(!cell.fsm().state_has_block(),
           "Cell@%s already in ekST_HAS_BLOCK",
           cell.loc().to_str().c_str());

  ER_CHECK(!cell.fsm().state_in_block_extent(),
           "Cell@%s already in ekST_BLOCK_EXTENT",
           cell.loc().to_str().c_str());
  return true;

error:
  return false;
} /* block_addition_cell_check() */

void structure3D::block_add(const crepr::base_block3D* block) {
  m_placed.push_back(block);
} /* block_add() */

NS_END(structure, silicon);
