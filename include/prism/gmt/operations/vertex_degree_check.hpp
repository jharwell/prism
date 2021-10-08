/**
 * \file vertex_degree_check.hpp
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

#include "cosm/repr/block_variant.hpp"

#include "prism/properties/gmt.hpp"
#include "prism/gmt/ds/connectivity_graph.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class vertex_degree_check
 * \ingroup gmt operations
 *
 * \brief Enforces graph properties related to min and max vertex degree from
 * \cite Harwell2022c-EGM
 */
class vertex_degree_check : public rer::client<vertex_degree_check> {
 public:
  explicit vertex_degree_check(const pgds::connectivity_graph* graph)
      : ER_CLIENT_INIT("prism.gmt.operations.vertex_degree_check"),
        mc_graph(graph) {}

  /* Not move/copy constructable/assignable by default */
  vertex_degree_check(const vertex_degree_check&) = delete;
  vertex_degree_check& operator=(const vertex_degree_check&) = delete;
  vertex_degree_check(vertex_degree_check&&) = delete;
  vertex_degree_check& operator=(vertex_degree_check&&) = delete;

  bool operator()(const pgds::connectivity_graph::vertex_descriptor& vd) const;

 private:
  /* clang-format off */
  const pgds::connectivity_graph* mc_graph;
  /* clang-format on */
};

NS_END(operations, gmt, prism);
