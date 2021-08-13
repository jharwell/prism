/**
 * \file spec_graph.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_SILICON_STRUCTURE_DS_SPEC_GRAPH_HPP_
#define INCLUDE_SILICON_STRUCTURE_DS_SPEC_GRAPH_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <filesystem>
#include <memory>
#include <boost/property_map/dynamic_property_map.hpp>

#include "rcppsw/er/client.hpp"
#include "rcppsw/ds/graph/hgrid3D.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"

#include "silicon/structure/repr/block_spec.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, ds);
namespace fs = std::filesystem;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class spec_graph
 * \ingroup structure ds
 *
 * \brief Graph containing the specification of the \ref structure3D to be
 * built, read from a file during initialization.
 */
class spec_graph : public rer::client<spec_graph>,
                   public rpdecorator::decorator<rdgraph::hgrid3D<
                                                   rdgraph::hgrid3D_spec<
                                                     /* Vertex properties type */
                                                     ssrepr::block_anchor_spec,
                                                     /* Edge properties type  */
                                                     ssrepr::block_extent_spec>
                                                   >
                                                 > {
 public:
  RCPPSW_DECORATE_DECL(spec_type);
  RCPPSW_DECORATE_DECL(view_type);

  /**
   * \brief Read the spec from the specified file.
   */
  static spec_graph from_file(const fs::path& path);

  ~spec_graph(void) override = default;

  RCPPSW_DECORATE_DECLDEF(operator[], const);
  RCPPSW_DECORATE_DECLDEF(size, const);
  RCPPSW_DECORATE_DECLDEF(subgraph, const);
  RCPPSW_DECORATE_DECLDEF(find, const);

  const ssrepr::block_anchor_spec* find(const rmath::vector3z& c) const {
    if (auto vd = decoratee().find(c)) {
      return &decoratee()[*vd];
    }
    return nullptr;
  }

 private:
  explicit spec_graph(const decoratee_type& decoratee);
};

NS_END(ds, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_DS_SPEC_GRAPH_HPP_ */
