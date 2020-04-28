/**
 * \file slice2D.cpp
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
#include "silicon/structure/slice2D.hpp"

#include <algorithm>

#include "silicon/structure/structure3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
slice2D::slice2D(const slice_coords& coords,
                 const structure3D* structure)
    : ER_CLIENT_INIT("silicon.structure.slice2D"),
      mc_coords(coords),
      mc_view(structure->subgrid(mc_coords.ll, mc_coords.ur)),
      mc_structure(structure) {
  ER_ASSERT(rmath::vector3z::X == mc_coords.axis || rmath::vector3z::Y == mc_coords.axis ||
            rmath::vector3z::Z == mc_coords.axis,
            "Bad slice axis %s",
            mc_coords.axis.to_str().c_str());
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool slice2D::is_hamiltonian(void) const {
  /* Using X,Y for simplicity, but holds for any 2D slice from a 3D grid. */
  size_t x = d1();
  size_t y = d2();

  /* 1 vertex graph is trivially Hamiltonian */
  if (1 == x && 1 == y) {
    return true;
  }
  /* All Hamiltonian graphs are biconnected, and a connected line is not */
  else if ((1 == x && y > 1) || (1 == y && x > 1)) {
    return false;
  } /* Any bipartite graph with unbalanced vertex parity is not Hamiltonian */
  else if ((1 == x % 2 && 1 == y % 2) && (1 != x * y)) {
    return false;
  }
  /*
   * If X or Y is even, and X > 1, Y > 1, the graph is Hamiltonian
   */
  else if (((0 == x % 2) || (0 == y % 2)) && (x > 1 && y > 1)) {
    return true;
  } else {
    ER_FATAL_SENTINEL("Unknown case for Hamiltonian test");
    return false;
  }
} /* is_hamiltonian() */

bool slice2D::is_traversable_d1(void) const {
  /*
   * Assume Z axis in the comments in this function, but the code should work
   * for any axis.
   */
  for (size_t j = 0; j < d2(); ++j) {
    for (size_t i = 0; i < d1(); ++i) {
      auto* spec = mc_structure->cell_spec_retrieve(access(i, j).loc());
      /*
       * Cells containing cube blocks can never result in discontinuity within
       * the mc_view.
       */
      if (crepr::block_type::ekCUBE == spec->block_type) {
        ER_TRACE("cell(%zu,%zu) contains cube--always traversable", i, j);
        continue;
      }

      if (crepr::block_type::ekRAMP == spec->block_type) {
        /*
         * Ramp block extends to the end of the row in X, so no more checking
         * needed
         */
        if (i + spec->extent >= d1()) {
          continue;
        }
        auto* spec2 = mc_structure->cell_spec_retrieve(access(i + spec->extent, j).loc());

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
          ER_TRACE(
              "cell(%zu%zu) contains ramp,cell(%zu+%zu=%zu,%zu) also "
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
        auto* spec2 = mc_structure->cell_spec_retrieve(access(i + spec->extent, j).loc());
        if ((crepr::block_type::ekRAMP == spec2->block_type &&
             rmath::radians::kZERO != spec2->z_rotation) ||
            cfsm::cell3D_state::ekST_BLOCK_EXTENT == spec2->state) {
          ER_TRACE(
              "cell(%zu%zu) contains cube, cell(%zu+%zu=%zu,%zu) also "
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
  }   /* for(j..) */
  return true;
} /* is_traversable_d1() */

bool slice2D::is_traversable_d2(void) const {
  /*
   * Assume Z axis in the comments in this function, but the code should work
   * for any axis.
   */
  for (size_t i = 0; i < d1(); ++i) {
    for (size_t j = 0; j < d2(); ++j) {
      auto* spec = mc_structure->cell_spec_retrieve(access(i, j).loc());
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
        auto* spec2 = mc_structure->cell_spec_retrieve(access(i, j + spec->extent).loc());

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
          ER_TRACE(
              "cell(%zu%zu) contains ramp,cell(%zu,%zu=%zu+%zu) also "
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
        auto* spec2 = mc_structure->cell_spec_retrieve(access(i, j + spec->extent).loc());
        if ((crepr::block_type::ekRAMP == spec2->block_type &&
             rmath::radians::kPI_OVER_TWO != spec2->z_rotation) ||
            cfsm::cell3D_state::ekST_BLOCK_EXTENT == spec2->state) {
          ER_TRACE(
              "cell(%zu%zu) contains cube, cell(%zu,%zu=%zu+%zu) also "
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
  }   /* for(j..) */
  return true;
} /* is_traversable_d2() */

bool slice2D::is_traversable(const rmath::radians& orientation) const {
  if (rmath::radians::kZERO == orientation) {
    return is_traversable_d1();
  } else if (rmath::radians::kPI_OVER_TWO == orientation) {
    return is_traversable_d2();
  }
  ER_FATAL_SENTINEL("Bad orientation for traversability: %s",
                    orientation.to_str().c_str());
  return false;
} /* is_traversable() */

bool slice2D::is_feasible(void) const {
  for (size_t i = 0; i < d1(); ++i) {
    for (size_t j = 0; j < d2(); ++j) {
      auto* spec = mc_structure->cell_spec_retrieve(access(i, j).loc());
      /*
       * You can't have cube blocks (or any other type of block) specified in
       * the cells that should be occupied by the ramp block extent).
       */
      if (crepr::block_type::ekRAMP == spec->block_type &&
          rmath::radians::kZERO == spec->z_rotation) {
        for (size_t m = 1; m < spec->extent; ++m) {
          auto* extent_spec = mc_structure->cell_spec_retrieve(access(i + m, j).loc());
          if (cfsm::cell3D_state::ekST_BLOCK_EXTENT != extent_spec->state) {
            return false;
          }
        } /* for(m..) */
      } else if (crepr::block_type::ekRAMP == spec->block_type &&
                 rmath::radians::kPI_OVER_TWO == spec->z_rotation) {
        for (size_t m = 1; m < spec->extent; ++m) {
          auto* extent_spec = mc_structure->cell_spec_retrieve(access(i, j + m).loc());
          if (cfsm::cell3D_state::ekST_BLOCK_EXTENT != extent_spec->state) {
            return false;
          }
        } /* for(m..) */
      } else if (crepr::block_type::ekRAMP == spec->block_type &&
                 rmath::radians::kPI == spec->z_rotation) {
        for (size_t m = 1; m < spec->extent; ++m) {
          auto* extent_spec = mc_structure->cell_spec_retrieve(access(i - m, j).loc());
          if (cfsm::cell3D_state::ekST_BLOCK_EXTENT != extent_spec->state) {
            return false;
          }
        } /* for(m..) */
      } else if (crepr::block_type::ekRAMP == spec->block_type &&
                 rmath::radians::kTHREE_PI_OVER_TWO == spec->z_rotation) {
        for (size_t m = 1; m < spec->extent; ++m) {
          auto* extent_spec = mc_structure->cell_spec_retrieve(access(i, j - m).loc());
          if (cfsm::cell3D_state::ekST_BLOCK_EXTENT != extent_spec->state) {
            return false;
          }
        } /* for(m..) */
      }
    } /* for(j..) */
  }   /* for(i..) */
  return true;
} /* is_feasible() */

bool slice2D::cell_is_exterior(const cds::cell3D& cell) const {
  bool exterior_d1 = RCSW_IS_BETWEEN(cell.loc().x(), 1, d1() - 1);
  bool exterior_d2 = RCSW_IS_BETWEEN(cell.loc().y(), 1, d2() - 1);
  return exterior_d1 && exterior_d2;
} /* cell_is_hole() */

bool slice2D::cell_is_simple_hole(const cds::cell3D& cell) const {
  auto* ijkminus1 = mc_structure->cell_spec_retrieve(cell.loc() - rmath::vector3z::Z);

  size_t face_count = 0;
  /*
   * If the cell is on the ground, then by definition its bottom face is being
   * used.
   */
  face_count += (0 == cell.loc().z());

  /*
   * If the cell at (i,j,k-1) is a cube block, then the bottom face is being
   * used. If it is part of a ramp block, then it is NOT being used.
   */
  face_count += (nullptr != ijkminus1 &&
                 cfsm::cell3D_state::ekST_HAS_BLOCK == ijkminus1->state &&
                 crepr::block_type::ekCUBE == ijkminus1->block_type);

  auto* spec = mc_structure->cell_spec_retrieve(cell.loc());
  return cfsm::cell3D_state::ekST_EMPTY == spec->state && face_count > 0;
} /* cell_is_simple_hole() */

std::set<const cds::cell3D*> slice2D::simple_holes(void) const {
  std::set<const cds::cell3D*> ret;
  for (size_t i = 0; i < d1(); ++i) {
    for (size_t j = 0; j < d2(); ++j) {
      if (cell_is_simple_hole(access(i, j))) {
        ER_TRACE("Cell@%s is simple hole", access(i, j).loc().to_str().c_str());
        ret.insert(&access(i, j));
      }
    } /* for(j..) */
  }   /* for(i..) */
  return ret;
} /* simple_holes() */

std::set<slice2D::topological_hole_type> slice2D::topological_holes(void) const {
  auto sholes = simple_holes();
  std::set<topological_hole_type> ret;
  ER_DEBUG("Slice contains %zu simple holes", sholes.size());
  /*
   * Iterate through all simple holes, and add all the simple holes which are
   * adjacent to a given one to a set: this is a topological hole. Different
   * simple holes can result in the same adjacency set, which is why we use a
   * set for the return type, because duplicates will be overwritten, and the
   * unique set of topological holes will be returned.
   */
  for (auto* shole1 : sholes) {
    std::set<const cds::cell3D*> thole = {shole1};
    for (auto* shole2 : sholes) {
      if (shole1 != shole2 && cells_are_adjacent(*shole1, *shole2)) {
        thole.insert(shole2);
      }
    } /* for(shole2..) */

    /*
     * If there is a path from any cell in the connected component to an
     * exterior node, then we don't have a hole from a topological point of
     * view (at least I don't think so...)
     */
    if (std::all_of(thole.begin(), thole.end(), [&](const cds::cell3D* cell) {
          return !cell_is_exterior(*cell);
        })) {
      ret.insert(thole);
    }
  } /* for(&shole..) */
  return ret;
} /* topological_holes() */

bool slice2D::cells_are_adjacent(const cds::cell3D& cell1,
                                 const cds::cell3D& cell2) const {
  bool d1plus1_neighbor, d1minus1_neighbor, d2plus1_neighbor, d2minus1_neighbor;
  if (rmath::vector3z::Z == mc_coords.axis) {
    d1plus1_neighbor = (cell1.loc() + rmath::vector3z::X == cell2.loc());
    d1minus1_neighbor = (cell1.loc() - rmath::vector3z::X == cell2.loc());
    d2plus1_neighbor = (cell1.loc() + rmath::vector3z::Y == cell2.loc());
    d2minus1_neighbor = (cell1.loc() - rmath::vector3z::Y == cell2.loc());
  } else if (rmath::vector3z::Y == mc_coords.axis) {
    d1plus1_neighbor = (cell1.loc() + rmath::vector3z::X == cell2.loc());
    d1minus1_neighbor = (cell1.loc() - rmath::vector3z::X == cell2.loc());
    d2plus1_neighbor = (cell1.loc() + rmath::vector3z::Z == cell2.loc());
    d2minus1_neighbor = (cell1.loc() - rmath::vector3z::Z == cell2.loc());
  } else {
    d1plus1_neighbor = (cell1.loc() + rmath::vector3z::Y == cell2.loc());
    d1minus1_neighbor = (cell1.loc() - rmath::vector3z::Y == cell2.loc());
    d2plus1_neighbor = (cell1.loc() + rmath::vector3z::Z == cell2.loc());
    d2minus1_neighbor = (cell1.loc() - rmath::vector3z::Z == cell2.loc());
  }

  return d1plus1_neighbor || d1minus1_neighbor || d2plus1_neighbor ||
         d2minus1_neighbor;
} /* cells_are_adjacent() */

size_t slice2D::d1(void) const {
  if (rmath::vector3z::X == mc_coords.axis) {
    return mc_view.shape()[1];
  } else {
    return mc_view.shape()[0];
  }
} /* d1() */

size_t slice2D::d2(void) const {
  if (rmath::vector3z::Z == mc_coords.axis) {
    return mc_view.shape()[1];
  } else {
    return mc_view.shape()[2];
  }
} /* d2() */

const cds::cell3D& slice2D::access(size_t d1, size_t d2) const {
  if (rmath::vector3z::X == mc_coords.axis) {
    return mc_view[0][d1][d2];
  } else if (rmath::vector3z::Y == mc_coords.axis) {
    return mc_view[d1][0][d2];
  } else {
    return mc_view[d1][d2][0];
  }
} /* access() */

bool slice2D::contains(const rmath::vector3z& coord) const {
  if (rmath::vector3z::X == mc_coords.axis) {
    return coord.x() == mc_coords.offset &&
        coord.y() < d1() && coord.z() < d2();
  } else if (rmath::vector3z::Y == mc_coords.axis) {
    return coord.y() == mc_coords.offset &&
        coord.x() < d1() && coord.z() < d2();
  } else {
    return coord.z() == mc_coords.offset &&
        coord.x() < d1() && coord.y() < d2();
  }
} /* contains() */

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
slice2D::slice_coords slice2D::coords_calc(const rmath::vector3z& axis,
                                           const structure3D* structure,
                                           size_t offset) {
  if (rmath::vector3z::X == axis) {
    return {
      axis,
          offset,
      {offset, 0, 0},
      {offset + 1, structure->ydsize(), structure->zdsize()}
    };
  } else if (rmath::vector3z::Y == axis) {
    return {
      axis,
          offset,
          {0, offset, 0},
          {structure->xdsize(), offset + 1, structure->zdsize()}
    };
  } else {
    return {
      axis,
          offset,
      {0, 0, offset},
      {structure->xdsize(), structure->ydsize(), offset + 1}
    };
  }
}

NS_END(structure, silicon);
