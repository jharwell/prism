/**
 * \file connectivity_check.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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
#include "prism/gmt/operations/connectivity_check.hpp"

#include "cosm/repr/base_block3D.hpp"

#include "prism/gmt/ds/connectivity_graph.hpp"
#include "prism/gmt/operations/vertex_degree_check.hpp"
#include "prism/gmt/operations/vertex_extent_check.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, operations);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool connectivity_check::operator()(const pgds::connectivity_graph* graph) const {
  auto degree_check = vertex_degree_check(graph);
  auto extent_check = vertex_extent_check(graph);

  /* First, per-vertex checks */
  auto [v_begin, v_end] = graph->vertices();
  for (auto vd = v_begin; vd != v_end; ++vd) {
    auto vertex = graph->operator[](*vd);

    /* Check vertex degree */
    ER_CHECK(degree_check(*vd),
             "Bad vertex degree for block@%s",
             rcppsw::to_string(vertex.coord).c_str());

    /* Check vertex extent */
    ER_CHECK(extent_check(*vd),
             "Bad vertex extent for block@%s",
             rcppsw::to_string(vertex.coord).c_str());
  } /* for(vd..) */
  ER_INFO("All connectivity per-vertex checks passed");

  return true;

error:
  return false;
} /* operator()() */


NS_END(operations, gmt, prism);
