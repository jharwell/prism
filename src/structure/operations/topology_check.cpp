/**
 * \file topology_check.cpp
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
#include "silicon/structure/operations/topology_check.hpp"

#include "silicon/structure/repr/vshell.hpp"
#include "silicon/structure/repr/ct_coord.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, operations);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool topology_check::operator()(const ssds::spec_graph* graph,
                                const ssrepr::vshell* vshell) const {
  /* PROPERTY: top layer has no topological holes */
  for (size_t z = 0; z < vshell->real()->zdsize() - 1; ++z) {
    auto slice = ssrepr::slice2D(ssrepr::slice2D::spec_calc(rmath::vector3z::Z,
                                                            z,
                                                            vshell),
                                 graph);
    ER_CHECK(layer_check(slice, graph), "Layer%zu failed validation", z);
    ER_DEBUG("Layer%zu OK", z);
  } /* for(z..) */
  ER_INFO("All layers validated");
  return true;

error:
  return false;
} /* operator()() */

bool topology_check::layer_check(const ssrepr::slice2D& layer,
                                 const ssds::spec_graph* graph) const {
  /*
   * @todo For right now, we disallow ALL holes in structures, because even
   * simple holes are not feasible unless beam blocks exist, which they do
   * not yet. This may be relaxed at some point in the future.
   */
  if (!topological_holes(layer).empty()) {
    ER_ERR("Layer contains one or more topological holes");
    return false;
  }

  return true;

error:
  return false;
} /* layer_check() */

std::vector<topology_check::simple_hole_type> topology_check::extent_holes(
    ssrepr::slice2D& layer,
    const ssrepr::vshell* vshell) const {
  std::vector<simple_hole_type> ret;

  auto [v_begin, v_end] = layer.vertices();
  for (auto vd = v_begin; vd != v_end; ++vd) {
    /*
     * Property 1: Extent holes cannot be anchor holes. Holds by default since
     * all vertices we iterate over are anchor points.
     */
    if (is_anchor_hole(layer.access(*vd), vshell)) {
      continue;
    }

    /*
     * Property 2: Extent holes are not crossed by exactly 1 edge. This is
     * verified by checking the edge weight (block extent) between two vertices
     * (u,v) matches the L1 norm distance between their anchor points.
     */
    auto [oe_begin, oe_end] = layer.out_edges(*vd);
    for (auto oed = oe_begin; oed != oe_end; ++oed) {
      auto adjacent = layer.access(layer.target(*oed));
      if (rmath::l1norm(layer.access(*vd)->coord,
                        adjacent->coord) == layer.access(*oed)->weight) {
        continue;
      }
      ret.push_back(layer.access(*vd));
    } /* for(oed..) */
  } /* (vd...) */


  return ret;
} /* extent_holes() */

bool topology_check::is_anchor_hole(const ssrepr::block_anchor_spec* spec,
                                    const ssrepr::vshell* vshell) const {
  /* Criteria 1: contains no anchor point */
  RCPPSW_CHECK(nullptr == spec);

  /* Criteria 2: Not an exterior cell */
  RCPPSW_CHECK(!on_exterior(spec, vshell));

  return true;

error:
  return false;
} /* is_anchor_hole() */

bool topology_check::on_exterior(const ssrepr::block_anchor_spec* spec,
                                 const ssrepr::vshell* vshell) const {
  return !spec->anchor.is_pd() ||
      spec->anchor.x() == vshell->real()->xdsize() - 1 ||
      spec->anchor.y() == vshell->real()->ydsize() - 1 ||
      spec->anchor.z() == vshell->real()->zdsize() - 1;
} /* on_exterior() */

/* std::set<topology_check::topological_hole_type> topology_check::topological_holes(void) const { */
/*   auto sholes = simple_holes(); */
/*   std::set<topological_hole_type> ret; */
/*   ER_DEBUG("Slice contains %zu simple holes", sholes.size()); */
/*   /\* */
/*    * Iterate through all simple holes, and add all the simple holes which are */
/*    * adjacent to a given one to a set: this is a topological hole. Different */
/*    * simple holes can result in the same adjacency set, which is why we use a */
/*    * set for the return type, because duplicates will be overwritten, and the */
/*    * unique set of topological holes will be returned. */
/*    *\/ */
/*   for (const auto* shole1 : sholes) { */
/*     std::set<const cds::cell3D*> thole = { shole1 }; */
/*     for (const auto* shole2 : sholes) { */
/*       if (shole1 != shole2 && cells_are_adjacent(*shole1, *shole2)) { */
/*         thole.insert(shole2); */
/*       } */
/*     } /\* for(shole2..) *\/ */

/*     /\* */
/*      * If there is a path from any cell in the connected component to an */
/*      * exterior node, then we don't have a hole from a topological point of */
/*      * view (at least I don't think so...) */
/*      *\/ */
/*     if (std::all_of(thole.begin(), thole.end(), [&](const cds::cell3D* cell) { */
/*           return !cell_is_exterior(*cell); */
/*         })) { */
/*       ret.insert(thole); */
/*     } */
/*   } /\* for(&shole..) *\/ */
/*   return ret; */
/* } /\* topological_holes() *\/ */

/* bool topology_check::cells_are_adjacent(const ssds::block_anchor_spec* cell1, */
/*                                  const ssds::block_anchor_spec* cell2) const { */
/*   bool d1plus1_neighbor, d1minus1_neighbor, d2plus1_neighbor, d2minus1_neighbor; */
/*   if (rmath::vector3z::Z == mc_coords.axis) { */
/*     d1plus1_neighbor = (cell1.loc() + rmath::vector3z::X == cell2.loc()); */
/*     d1minus1_neighbor = (cell1.loc() - rmath::vector3z::X == cell2.loc()); */
/*     d2plus1_neighbor = (cell1.loc() + rmath::vector3z::Y == cell2.loc()); */
/*     d2minus1_neighbor = (cell1.loc() - rmath::vector3z::Y == cell2.loc()); */
/*   } else if (rmath::vector3z::Y == mc_coords.axis) { */
/*     d1plus1_neighbor = (cell1.loc() + rmath::vector3z::X == cell2.loc()); */
/*     d1minus1_neighbor = (cell1.loc() - rmath::vector3z::X == cell2.loc()); */
/*     d2plus1_neighbor = (cell1.loc() + rmath::vector3z::Z == cell2.loc()); */
/*     d2minus1_neighbor = (cell1.loc() - rmath::vector3z::Z == cell2.loc()); */
/*   } else { */
/*     d1plus1_neighbor = (cell1.loc() + rmath::vector3z::Y == cell2.loc()); */
/*     d1minus1_neighbor = (cell1.loc() - rmath::vector3z::Y == cell2.loc()); */
/*     d2plus1_neighbor = (cell1.loc() + rmath::vector3z::Z == cell2.loc()); */
/*     d2minus1_neighbor = (cell1.loc() - rmath::vector3z::Z == cell2.loc()); */
/*   } */

/*   return d1plus1_neighbor || d1minus1_neighbor || d2plus1_neighbor || */
/*          d2minus1_neighbor; */
/* } /\* cells_are_adjacent() *\/ */

NS_END(operations, structure, silicon);
