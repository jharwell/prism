/**
 * \file regularity_check.cpp
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
#include "prism/gmt/operations/regularity_check.hpp"

#include "cosm/repr/base_block3D.hpp"

#include "prism/gmt/ds/connectivity_graph.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, operations);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool regularity_check::operator()(const pgds::connectivity_graph* graph) const {
  auto [v_begin, v_end] = graph->vertices();
  for (auto vd = v_begin; vd != v_end; ++vd) {
    /* Check vertex type */

    auto vertex = graph->operator[](*vd);
    ER_CHECK(RCPPSW_IS_BETWEEN(vertex.block->md()->type(),
                               crepr::block_type::ekNONE,
                               crepr::block_type::ekRAMP),
             "Bad vertex type for block@%s",
             rcppsw::to_string(vertex.coord).c_str());
  } /* for(vd..) */
  ER_INFO("Regularity check passed");
  return true;

error:
  return false;
} /* operator()() */


NS_END(operations, gmt, prism);
