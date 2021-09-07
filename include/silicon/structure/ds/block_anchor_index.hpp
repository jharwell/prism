/**
 * \file block_anchor_index.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_DS_BLOCK_ANCHOR_INDEX_HPP_
#define INCLUDE_SILICON_STRUCTURE_DS_BLOCK_ANCHOR_INDEX_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/decorator/decorator.hpp"
#include "rcppsw/ds/rtree.hpp"

#include "silicon/silicon.hpp"
#include "silicon/structure/repr/block_spec.hpp"
#include "silicon/structure/ds/connectivity_graph.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, ds, detail);

using rtree_spec = rds::rtree_spec<rmath::vector3z,
                                   rds::rtree_point<rmath::vector3z>,
                                   connectivity_graph::vertex_descriptor>;
NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_anchor_index
 * \ingroup structure ds
 *
 * \brief Indexs a location in the \ref sstructure3D to its \ref
 * ssrepr::block_anchor_spec.
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

NS_END(ds, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_DS_BLOCK_ANCHOR_INDEX_HPP_ */
