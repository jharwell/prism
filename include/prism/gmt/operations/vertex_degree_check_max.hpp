/**
 * \file vertex_degree_check_max.hpp
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
#include "cosm/repr/block_variant.hpp"

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
 * \class vertex_degree_check_max
 * \ingroup gmt operations
 *
 * \brief Check the maximum degree for a given vertex in \ref
 * connectivity_graph, given what type of block it is associated with.
 */
class vertex_degree_check_max : public rer::client<vertex_degree_check_max> {
 public:
  explicit vertex_degree_check_max(const pgds::connectivity_graph* graph)
      : ER_CLIENT_INIT("prism.gmt.operations.vertex_degree_check_max"),
        mc_graph(graph) {}

  /* Not move/copy constructable/assignable by default */
  vertex_degree_check_max(const vertex_degree_check_max&) = delete;
  vertex_degree_check_max& operator=(const vertex_degree_check_max&) = delete;
  vertex_degree_check_max(vertex_degree_check_max&&) = delete;
  vertex_degree_check_max& operator=(vertex_degree_check_max&&) = delete;

  bool operator()(const pgds::connectivity_graph::vertex_descriptor& vd) const {
    auto vertex = mc_graph->operator[](vd);
    auto variant = crepr::make_variant(vertex.block);
    auto degree = mc_graph->degree(vd);
    auto max_degree = std::visit(max_degree_extract(), variant);
    ER_CHECK(degree <= max_degree,
             "Vertex@%s has too many neighbors: %zu < %zu",
             rcppsw::to_string(vertex.coord).c_str(),
             degree,
             max_degree);
    return true;

 error:
    return false;
  }

 private:
  struct max_degree_extract {
    size_t operator()(const crepr::cube_block3D*) const {
      return ppgmt::kMAX_MANHATTAN_NEIGHBORS_CUBE;
    }
    size_t operator()(const crepr::ramp_block3D*) const {
      return ppgmt::kMAX_MANHATTAN_NEIGHBORS_RAMP;
    }
  };

  /* clang-format off */
  const pgds::connectivity_graph* mc_graph;
  /* clang-format on */
};

NS_END(operations, gmt, prism);
