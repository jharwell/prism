/**
 * \file traversability_check.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include "silicon/structure/operations/traversability_check.hpp"

#include "silicon/structure/repr/ct_coord.hpp"
#include "silicon/structure/repr/vshell.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, operations);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool traversability_check::operator()(const ssds::spec_graph* spec,
                                      const ssrepr::vshell* vshell,
                                      const rmath::radians& orientation) const {
  /* PROPERTY: top layer is ALWAYS traversable */
  for (size_t z = 0; z < vshell->real()->zdsize() - 1; ++z) {
    auto slice = ssrepr::slice2D(ssrepr::slice2D::spec_calc(rmath::vector3z::Z,
                                                            z,
                                                            spec,
                                                            vshell),
                                 spec);
    if (rmath::radians::kZERO == orientation) {
      RCPPSW_CHECK(is_traversable_d1_neg());
    } else if (rmath::radians::kPI_OVER_TWO == orientation) {
      RCPPSW_CHECK(is_traversable_d2_neg());
    } else if (rmath::radians::kPI == orientation) {
      RCPPSW_CHECK(is_traversable_d1_pos());
    } else if (rmath::radians::kTHREE_PI_OVER_TWO == orientation) {
      RCPPSW_CHECK(is_traversable_d2_pos());
    }
    ER_DEBUG("Layer%zu OK", z);
  } /* for(z..) */

  return true;

error:
  return false;
} /* operator()() */

bool traversability_check::is_traversable_d1_neg(void) const {
  /*
   * Assume Z for slice axis in the comments in this function, but the code
   * should work for any axis.
   */
  for (size_t j = 0; j < mc_layer.d2(); ++j) {
    for (size_t i = 0; i < mc_layer.d1(); ++i) {
      auto coord = ssds::ct_coord{ rmath::vector3z{i, j},
                                   ssds::ct_coord::relativity::ekVORIGIN,
                                   mc_structure };
      const auto* spec = mc_graph->find(coord);
      /*
       * Vertices containing cube blocks within the layer can never result in
       * discontinuity and non-traversability.
       */
      if (crepr::block_type::ekCUBE == spec->block_type) {
        ER_TRACE("Vertex(%zu,%zu) contains cube--always traversable", i, j);
        continue;
      }

      auto ext_coord = ssds::ct_coord{ access(i + spec->extent, j).loc(),
                                       ssds::ct_coord::relativity::ekVORIGIN,
                                       mc_structure };

      if (crepr::block_type::ekRAMP == spec->block_type) {
        /*
         * Ramp block extends to the end of the row in X, so no more checking
         * needed
         */
        if (i + spec->extent >= d1()) {
          continue;
        }
        const auto* spec2 = mc_structure->cell_spec_retrieve(ext_coord);

        /*
         * Ramp blocks, if they are oriented along the X axis (in either
         * direction), will cause discontinuities if we encounter either:
         *
         * - Two adjacent blocks at x, x+block_extent+1 oriented in the SAME
         *   direction
         * - Two blocks oriented in OPPOSITE directions without either an empty
         *   cell or cube block in between, depending.
         */
        if (crepr::block_type::ekRAMP == spec2->block_type) {
          ER_TRACE("cell(%zu%zu) contains ramp,cell(%zu+%zu=%zu,%zu) also "
                   "contains ramp",
                   i,
                   j,
                   i,
                   spec->extent,
                   i + spec->extent,
                   j);

          return false;
        }
      }
      /*
       * If there is a cube block in (i,j) then discontinuities occur if we
       * encounter either:
       *
       * - A ramp block oriented in any other way than +x in cell
       *   (i+cube_block_extent, j).
       *
       * - A ramp block extent cell in (i+cube_block_extent,j)
       */
      else if (crepr::block_type::ekCUBE == spec->block_type &&
               cell_is_exterior(access(i, j))) {
        const auto* spec2 = mc_structure->cell_spec_retrieve(ext_coord);
        if ((crepr::block_type::ekRAMP == spec2->block_type &&
             rmath::radians::kZERO != spec2->z_rotation) ||
            cfsm::cell3D_state::ekST_BLOCK_EXTENT == spec2->state) {
          ER_TRACE("cell(%zu%zu) contains cube, cell(%zu+%zu=%zu,%zu) also "
                   "ramp not oriented in +x",
                   i,
                   j,
                   i,
                   spec->extent,
                   i + spec->extent,
                   j);
          return false;
        }
      }
    } /* for(i..) */
  } /* for(j..) */
  return true;
} /* is_traversable_d1_neg() */

bool traversability_check::is_traversable_d1_pos(void) const {
  /*
   * Assume Z axis in the comments in this function, but the code should work
   * for any axis.
   */
  for (int j = d2() - 1; j >= 0; --j) {
    for (int i = d1() - 1; i >= 0; --i) {
      auto coord = ssds::ct_coord{ access(i, j).loc(),
                                   ssds::ct_coord::relativity::ekVORIGIN,
                                   mc_structure };
      const auto* spec = mc_structure->cell_spec_retrieve(coord);
      /*
       * Cells containing cube blocks can never result in discontinuity within
       * the mc_view.
       */
      if (crepr::block_type::ekCUBE == spec->block_type) {
        ER_TRACE("cell(%d,%d) contains cube--always traversable", i, j);
        continue;
      }

      auto ext_coord = ssds::ct_coord{ access(i - spec->extent, j).loc(),
                                       ssds::ct_coord::relativity::ekVORIGIN,
                                       mc_structure };

      if (crepr::block_type::ekRAMP == spec->block_type) {
        /*
         * Ramp block extends to the end of the row in X, so no more checking
         * needed
         */
        if (i - spec->extent <= 0) {
          continue;
        }
        const auto* spec2 = mc_structure->cell_spec_retrieve(ext_coord);

        /*
         * Ramp blocks, if they are oriented along the X axis (in either
         * direction), will cause discontinuities if we encounter either:
         *
         * - Two adjacent blocks at x, x-block_extent-1 oriented in the SAME
         *   direction
         * - Two blocks oriented in OPPOSITE directions without either an empty
         *   cell or cube block in between, depending.
         */
        if (crepr::block_type::ekRAMP == spec2->block_type) {
          ER_TRACE("cell(%d,%d) contains ramp,cell(%d-%lu=%lu,%d) also "
                   "contains ramp",
                   i,
                   j,
                   i,
                   spec->extent,
                   i - spec->extent,
                   j);

          return false;
        }
      }
      /*
       * If there is a cube block in (i,j) then discontinuities occur if we
       * encounter either:
       *
       * - A ramp block oriented in any other way than -x in cell
       *   (i-cube_block_extent, j).
       *
       * - A ramp block extent cell in (i-cube_block_extent,j)
       */
      else if (crepr::block_type::ekCUBE == spec->block_type &&
               cell_is_exterior(access(i, j))) {
        const auto* spec2 = mc_structure->cell_spec_retrieve(ext_coord);
        if ((crepr::block_type::ekRAMP == spec2->block_type &&
             rmath::radians::kZERO != spec2->z_rotation) ||
            cfsm::cell3D_state::ekST_BLOCK_EXTENT == spec2->state) {
          ER_TRACE("cell(%d,%d) contains cube, cell(%d-%lu=%lu,%d) also "
                   "ramp not oriented in -x",
                   i,
                   j,
                   i,
                   spec->extent,
                   i - spec->extent,
                   j);
          return false;
        }
      }
    } /* for(i..) */
  } /* for(j..) */
  return true;
} /* is_traversable_d1_pos() */

bool traversability_check::is_traversable_d2_neg(void) const {
  /*
   * Assume Z axis in the comments in this function, but the code should work
   * for any axis.
   */
  for (size_t i = 0; i < d1(); ++i) {
    for (size_t j = 0; j < d2(); ++j) {
      auto coord = ssds::ct_coord{ access(i, j).loc(),
                                   ssds::ct_coord::relativity::ekVORIGIN,
                                   mc_structure };
      const auto* spec = mc_structure->cell_spec_retrieve(coord);
      /*
       * Cells containing cube blocks can never result in discontinuity within
       * the mc_view.
       */
      if (crepr::block_type::ekCUBE == spec->block_type) {
        continue;
      }
      if (crepr::block_type::ekRAMP == spec->block_type) {
        /*
         * Ramp block extends to the end of the row in Y, so no more checking
         * needed
         */
        if (j + spec->extent >= d2()) {
          continue;
        }
        auto ext_coord = ssds::ct_coord{ access(i, j + spec->extent).loc(),
                                         ssds::ct_coord::relativity::ekVORIGIN,
                                         mc_structure };
        const auto* spec2 = mc_structure->cell_spec_retrieve(ext_coord);

        /*
         * Ramp blocks, if they are oriented along the Y axis (in either
         * direction), will cause discontinuities if we encounter either:
         *
         * - Two adjacent blocks at y, y+block_extent+1 oriented in the SAME
         *   direction
         * - Two blocks oriented in OPPOSITE directions without either an empty
         *   cell or cube block in between, depending.
         */
        if (crepr::block_type::ekRAMP == spec2->block_type) {
          ER_TRACE("cell(%zu%zu) contains ramp,cell(%zu,%zu=%zu+%zu) also "
                   "contains ramp",
                   i,
                   j,
                   i,
                   j + spec->extent,
                   j,
                   spec->extent);

          return false;
        }
      }
      /*
       * If there is a cube block in (i,j) then discontinuities occur if we
       * encounter either:
       *
       * - A ramp block oriented in any other way than +y in cell
       *   (i, j+cube_block_extent).
       *
       * - A ramp block extent cell in (i, j+cube_block_extent)
       */
      else if (crepr::block_type::ekCUBE == spec->block_type &&
               cell_is_exterior(access(i, j))) {
        auto ext_coord = ssds::ct_coord{ access(i, j + spec->extent).loc(),
                                         ssds::ct_coord::relativity::ekVORIGIN,
                                         mc_structure };
        const auto* spec2 = mc_structure->cell_spec_retrieve(ext_coord);
        if ((crepr::block_type::ekRAMP == spec2->block_type &&
             rmath::radians::kPI_OVER_TWO != spec2->z_rotation) ||
            cfsm::cell3D_state::ekST_BLOCK_EXTENT == spec2->state) {
          ER_TRACE("cell(%zu%zu) contains cube, cell(%zu,%zu=%zu+%zu) also "
                   "ramp not oriented in +x",
                   i,
                   j,
                   i,
                   j + spec->extent,
                   j,
                   spec->extent);
          return false;
        }
      }
    } /* for(i..) */
  } /* for(j..) */
  return true;
} /* is_traversable_d2_neg() */

bool traversability_check::is_traversable_d2_pos(void) const {
  /*
   * Assume Z axis in the comments in this function, but the code should work
   * for any axis.
   */
  for (int i = d1() - 1; i >= 0; --i) {
    for (int j = d2() - 1; j >= 0; --j) {
      auto coord = ssds::ct_coord{ access(i, j).loc(),
                                   ssds::ct_coord::relativity::ekVORIGIN,
                                   mc_structure };
      const auto* spec = mc_structure->cell_spec_retrieve(coord);

      /*
       * Cells containing cube blocks can never result in discontinuity within
       * the mc_view.
       */
      if (crepr::block_type::ekCUBE == spec->block_type) {
        continue;
      }
      if (crepr::block_type::ekRAMP == spec->block_type) {
        /*
         * Ramp block extends to the end of the row in Y, so no more checking
         * needed
         */
        if (j - spec->extent <= 0) {
          continue;
        }
        auto ext_coord = ssds::ct_coord{ access(i, j - spec->extent).loc(),
                                         ssds::ct_coord::relativity::ekVORIGIN,
                                         mc_structure };
        const auto* spec2 = mc_structure->cell_spec_retrieve(ext_coord);

        /*
         * Ramp blocks, if they are oriented along the Y axis (in either
         * direction), will cause discontinuities if we encounter either:
         *
         * - Two adjacent blocks at y, y-block_extent-1 oriented in the SAME
         *   direction
         * - Two blocks oriented in OPPOSITE directions without either an empty
         *   cell or cube block in between, depending.
         */
        if (crepr::block_type::ekRAMP == spec2->block_type) {
          ER_TRACE("cell(%d,%d) contains ramp,cell(%d,%lu=%d-%zu) also "
                   "contains ramp",
                   i,
                   j,
                   i,
                   j - spec->extent,
                   j,
                   spec->extent);

          return false;
        }
      }
      /*
       * If there is a cube block in (i,j) then discontinuities occur if we
       * encounter either:
       *
       * - A ramp block oriented in any other way than -y in cell
       *   (i, j-cube_block_extent).
       *
       * - A ramp block extent cell in (i, j-cube_block_extent)
       */
      else if (crepr::block_type::ekCUBE == spec->block_type &&
               cell_is_exterior(access(i, j))) {
        auto ext_coord = ssds::ct_coord{ access(i, j + spec->extent).loc(),
                                         ssds::ct_coord::relativity::ekVORIGIN,
                                         mc_structure };
        const auto* spec2 = mc_structure->cell_spec_retrieve(ext_coord);
        if ((crepr::block_type::ekRAMP == spec2->block_type &&
             rmath::radians::kPI_OVER_TWO != spec2->z_rotation) ||
            cfsm::cell3D_state::ekST_BLOCK_EXTENT == spec2->state) {
          ER_TRACE("cell(%d,%d) contains cube, cell(%d,%lu=%d-%zu) also "
                   "ramp not oriented in -x",
                   i,
                   j,
                   i,
                   j - spec->extent,
                   j,
                   spec->extent);
          return false;
        }
      }
    } /* for(i..) */
  } /* for(j..) */
  return true;
} /* is_traversable_d2_pos() */

NS_END(operations, structure, silicon);
