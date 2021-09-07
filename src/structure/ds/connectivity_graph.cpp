/**
 * \file connectivity_graph.cpp
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
#include "silicon/structure/ds/connectivity_graph.hpp"

#include <fstream>

#include "rcppsw/ds/graph/graphml.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, ds);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
connectivity_graph::connectivity_graph(const decoratee_type& decoratee)
    : ER_CLIENT_INIT("silicon.structure.ds.connectivity_graph"),
      decorator(decoratee) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
connectivity_graph connectivity_graph::from_file(const fs::path& path) {
  /* build GraphML properties to be parsed */
  auto build_map = [&](boost::dynamic_properties& dp, auto& g) {
                     dp.property("type",
                                 boost::get(&ssrepr::block_anchor_spec::type, g));
                     dp.property("anchor",
                                 boost::get(&ssrepr::block_anchor_spec::anchor, g));
                     dp.property("z_rot",
                                 boost::get(&ssrepr::block_anchor_spec::z_rot, g));
                     dp.property("weight",
                                 boost::get(&ssrepr::block_extent_spec::weight, g));
               };
  decoratee_type decoratee;
  auto dp = rdgraph::property_map_gen<boost::dynamic_properties>(decoratee,
                                                                 build_map);
  /* read in the spec from GraphML into the decoratee (underlying graph) */
  rdgraph::read_graphml(path, decoratee, dp);
  return connectivity_graph(decoratee);
} /* from_file() */


NS_END(ds, structure, silicon);
