/**
 * \file vertex_extent_check.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include "prism/gmt/operations/vertex_extent_check.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, operations);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool vertex_extent_check::operator()(
    const pgds::connectivity_graph::vertex_descriptor& vd) const {
  /*
   * PROPERTY: Extent holes are not crossed by exactly 1 edge. This is verified
   * by checking the edge weight (block extent) between two vertices (u,v)
   * matches the L1 norm distance between their anchor points.
   */
  auto [oe_begin, oe_end] = mc_graph->out_edges(vd);
  auto vertex = mc_graph->operator[](vd);

  for (auto oed = oe_begin; oed != oe_end; ++oed) {
    auto adjacent = mc_graph->operator[](mc_graph->target(*oed));
    auto norm = rmath::l1norm(vertex.coord,
                              adjacent.coord);

    auto extent = mc_graph->operator[](*oed).weight;
    ER_CHECK(norm == extent,
             "Edge length mismatch %s->%s: %zu vs %zu -> incoherent extents",
             rcppsw::to_string(vertex.coord).c_str(),
             rcppsw::to_string(adjacent.coord).c_str(),
             norm,
             extent);
  } /* for(oed..) */

  ER_DEBUG("Vertex %s has coherent extents",
           rcppsw::to_string(vertex.coord).c_str());
  return true;

error:
  return false;
} /* operator()() */

NS_END(operations, gmt, prism);
