/**
 * \file connectivity_graph.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_PRISM_GMT_DS_CONNECTIVITY_GRAPH_HPP_
#define INCLUDE_PRISM_GMT_DS_CONNECTIVITY_GRAPH_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <filesystem>
#include <memory>
#include <boost/property_map/dynamic_property_map.hpp>

#include "rcppsw/er/client.hpp"
#include "rcppsw/ds/graph/hgrid3D.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"

#include "prism/gmt/repr/block_spec.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, ds);
namespace fs = std::filesystem;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class connectivity_graph
 * \ingroup gmt ds
 *
 * \brief Graph containing the connectivity specification of the \ref
 * spc_gmt to be built, read from a file during initialization.
 */
class connectivity_graph : public rer::client<connectivity_graph>,
                   public rpdecorator::decorator<rdgraph::hgrid3D<
                                                   rdgraph::hgrid3D_spec<
                                                     /* Vertex properties type */
                                                     pgrepr::block_anchor_spec,
                                                     /* Edge properties type  */
                                                     pgrepr::block_extent_spec>
                                                   >
                                                 > {
 public:
  RCPPSW_DECORATE_DECL(spec_type);
  RCPPSW_DECORATE_DECL(view_type);
  RCPPSW_DECORATE_DECL(vertex_descriptor);

  /**
   * \brief Read the spec from the specified file.
   */
  static connectivity_graph from_file(const fs::path& path);

  ~connectivity_graph(void) override = default;

  RCPPSW_DECORATE_DECLDEF(operator[], const);
  RCPPSW_DECORATE_DECLDEF(n_vertices, const);
  RCPPSW_DECORATE_DECLDEF(subgraph, const);
  RCPPSW_DECORATE_DECLDEF(find, const);
  RCPPSW_DECORATE_DECLDEF(contains, const);

  RCPPSW_DECORATE_DECLDEF(operator[]);

  pgrepr::block_anchor_spec* to_spec(const boost::optional<vertex_descriptor>& vd) {
    if (vd) {
      return to_spec(*vd);
    }
    return nullptr;
  }

  const pgrepr::block_anchor_spec* to_spec(const boost::optional<vertex_descriptor>& vd) const {
    return (const_cast<connectivity_graph*>(this))->to_spec(vd);
  }

  pgrepr::block_anchor_spec* to_spec(const vertex_descriptor& vd) { return &decoratee()[vd]; }
  const pgrepr::block_anchor_spec* to_spec(const vertex_descriptor& vd) const {
    return &decoratee()[vd];
  }

 private:
  explicit connectivity_graph(const decoratee_type& decoratee);
};

NS_END(ds, gmt, prism);

#endif /* INCLUDE_PRISM_GMT_DS_CONNECTIVITY_GRAPH_HPP_ */
