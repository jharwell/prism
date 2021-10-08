/**
 * \file block_anchor_index.hpp
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
#include "rcppsw/patterns/decorator/decorator.hpp"
#include "rcppsw/ds/rtree.hpp"

#include "prism/prism.hpp"
#include "prism/gmt/repr/block_spec.hpp"
#include "prism/gmt/ds/connectivity_graph.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, ds, detail);

using rtree_spec = rds::rtree_spec<rmath::vector3z,
                                   rds::rtree_point<rmath::vector3z>,
                                   connectivity_graph::vertex_descriptor>;
NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_anchor_index
 * \ingroup gmt ds
 *
 * \brief Indexs a location in the \ref sspc_gmt to its \ref
 * pgrepr::block_anchor_spec.
 */

class block_anchor_index : public rpdecorator::decorator<
  rds::rtree<detail::rtree_spec>
  > {
 public:
  RCPPSW_DECORATE_DECLDEF(begin, const);
  RCPPSW_DECORATE_DECLDEF(end, const);
  RCPPSW_DECORATE_DECLDEF(size, const);
  RCPPSW_DECORATE_DECLDEF(nearest, const);

  RCPPSW_DECORATE_DECLDEF(insert);
  RCPPSW_DECORATE_DECLDEF(begin);
  RCPPSW_DECORATE_DECLDEF(end);
};

NS_END(ds, gmt, prism);

