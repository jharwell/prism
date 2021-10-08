/**
 * \file vertex_degree_check.cpp
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
#include "prism/gmt/operations/vertex_degree_check.hpp"

#include "prism/gmt/operations/vertex_degree_check_min.hpp"
#include "prism/gmt/operations/vertex_degree_check_max.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, operations);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool vertex_degree_check::operator()(
    const pgds::connectivity_graph::vertex_descriptor& vd) const {
  auto max_check = vertex_degree_check_max(mc_graph);
  auto min_check = vertex_degree_check_min(mc_graph);

  RCPPSW_CHECK(max_check(vd));
  RCPPSW_CHECK(min_check(vd));

  return true;
error:
  return false;
} /* operator()() */

NS_END(operations, gmt, prism);
