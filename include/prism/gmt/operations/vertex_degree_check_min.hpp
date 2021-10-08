/**
 * \file vertex_degree_check_min.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"

#include "prism/gmt/ds/connectivity_graph.hpp"
#include "prism/properties/gmt.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class vertex_degree_check_min
 * \ingroup gmt operations
 *
 * \brief Check the minimum degree for a given vertex in \ref connectivity_graph.
 */
class vertex_degree_check_min : public rer::client<vertex_degree_check_min> {
 public:
  explicit vertex_degree_check_min(const pgds::connectivity_graph* graph)
      : ER_CLIENT_INIT("prism.gmt.operations.vertex_degree_check_min"),
        mc_graph(graph) {}

  /* Not move/copy constructable/assignable by default */
  vertex_degree_check_min(const vertex_degree_check_min&) = delete;
  vertex_degree_check_min& operator=(const vertex_degree_check_min&) = delete;
  vertex_degree_check_min(vertex_degree_check_min&&) = delete;
  vertex_degree_check_min& operator=(vertex_degree_check_min&&) = delete;

  bool operator()(const pgds::connectivity_graph::vertex_descriptor& vd) const {
    auto v = mc_graph->operator[](vd);
    auto degree = mc_graph->degree(vd);

    if (mc_graph->contains(v.coord + rmath::vector3z::Z)) {
      ER_CHECK(degree >= ppgmt::kABOVE_GROUND_MIN_MANHATTAN_NEIGHBORS,
               "Vertex@%s has too few neighbors: %zu < %zu",
               rcppsw::to_string(v.coord).c_str(),
               degree,
               ppgmt::kABOVE_GROUND_MIN_MANHATTAN_NEIGHBORS);
    }
    return true;

 error:
    return false;
  }

 private:
  /* clang-format off */
  const pgds::connectivity_graph* mc_graph;
  /* clang-format on */
};

NS_END(operations, gmt, prism);
