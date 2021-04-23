/**
 * \file validate_layer_spec.cpp
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
#include "silicon/structure/operations/validate_spec_composability.hpp"

#include "silicon/structure/ds/ct_coord.hpp"
#include "silicon/structure/structure3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, operations);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool validate_spec_composability::operator()() const {
  for (size_t z = 0; z < mc_structure->zdsize() - 1; ++z) {
    auto lower = slice2D(
        slice2D::coords_calc(rmath::vector3z::Z, mc_structure, z), mc_structure);
    auto upper =
        slice2D(slice2D::coords_calc(rmath::vector3z::Z, mc_structure, z + 1),
                mc_structure);
    ER_CHECK(is_composable(lower, upper),
             "Layer%zu->layer%zu not composable",
             z,
             z + 1);
    ER_DEBUG("Layer%zu->layer%zu OK", z, z + 1);
  } /* for(z..) */
  return true;

error:
  return false;
} /* operator()() */

bool validate_spec_composability::is_composable(const slice2D& lower,
                                                const slice2D& upper) const {
  /*
   * @todo For right now, we disallow ALL holes in structures, because even
   * sibreample holes are not feasible unless beam blocks exist, which they do
   * not yet. This may be relaxed at some point in the future.
   */
  if (!lower.topological_holes().empty() || !upper.topological_holes().empty()) {
    ER_ERR("One or both layers contain topological holes");
    return false;
  }
  for (size_t i = 0; i < lower.d1(); ++i) {
    for (size_t j = 0; j < lower.d2(); ++j) {
      auto coordl = ssds::ct_coord{ lower.access(i, j).loc(),
                                    ssds::ct_coord::relativity::ekVORIGIN,
                                    mc_structure };
      auto coordu = ssds::ct_coord{ upper.access(i, j).loc(),
                                    ssds::ct_coord::relativity::ekVORIGIN,
                                    mc_structure };
      const auto* specl = mc_structure->cell_spec_retrieve(coordl);
      const auto* specu = mc_structure->cell_spec_retrieve(coordu);
      /*
       * If the lower layer cell at (i,j) contained a cube, you can have
       * anything in the upper layer cell at (i,j).
       */
      if (cfsm::cell3D_state::ekST_HAS_BLOCK == specl->state &&
          crepr::block_type::ekCUBE == specl->block_type) {
        continue;
      }

      if (cfsm::cell3D_state::ekST_HAS_BLOCK == specl->state &&
          crepr::block_type::ekRAMP == specl->block_type) {
        /*
         * If the lower layer cell at (i,j) contained a ramp block, the upper
         * layer MUST be empty (otherwise not physically feasible to stack the
         * layers, regardless of block type/orientation in the upper layer).
         */
        if (cfsm::cell3D_state::ekST_EMPTY != specu->state) {
          return false;
        }
        /*
         * For each cell in the lower layer that is part of the ramp block's
         * extent, the corresponding cells directly above each of them in the
         * upper layer must be empty (otherwise not physically feasible to stack
         * the layers, regardless of block type/orientation in the upper layer).
         */
        for (size_t m = 1; m < specl->extent; ++m) {
          const auto* spec_extent = mc_structure->cell_spec_retrieve(coordu);
          if (cfsm::cell3D_state::ekST_EMPTY != spec_extent->state) {
            return false;
          }
        } /* for(m..) */
      }
    } /* for(j..) */
  } /* for(i..) */
  return true;
} /* is_composable() */

NS_END(operations, structure, silicon);
