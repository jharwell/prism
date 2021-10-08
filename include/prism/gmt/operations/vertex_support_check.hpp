/**
 * \file vertex_support_check.hpp
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
 * \class vertex_support_check
 * \ingroup gmt operations
 *
 * \brief Check that all vertices with Z>0 have support from below.
 */
class vertex_support_check : public rer::client<vertex_support_check> {
 public:
  explicit vertex_support_check(const pgds::connectivity_graph* graph)
      : ER_CLIENT_INIT("prism.gmt.operations.vertex_support_check"),
        mc_graph(graph) {}

  /* Not move/copy constructable/assignable by default */
  vertex_support_check(const vertex_support_check&) = delete;
  vertex_support_check& operator=(const vertex_support_check&) = delete;
  vertex_support_check(vertex_support_check&&) = delete;
  vertex_support_check& operator=(vertex_support_check&&) = delete;

  bool operator()(const pgds::connectivity_graph::vertex_descriptor& vd) const {
    auto zplus1 = mc_graph->operator[](vd).coord;
    auto z = zplus1 - rmath::vector3z::Z;

    if (z.z() > 0) { /* not on ground */
      ER_CHECK(boost::none != mc_graph->find(z),
               "Vertex@%s does not have support below: no anchor@%s",
               rcppsw::to_string(zplus1).c_str(),
               rcppsw::to_string(z).c_str());
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
