/**
 * \file builder_los.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#ifndef INCLUDE_PRISM_REPR_BUILDER_LOS_HPP_
#define INCLUDE_PRISM_REPR_BUILDER_LOS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ds/block3D_vector.hpp"
#include "cosm/repr/graph3D_los.hpp"

#include "prism/gmt/repr/block_spec.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, repr);

using los_spec = rdgraph::hgrid3D_spec<pgrepr::block_anchor_spec,
                                       pgrepr::block_extent_spec>;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class builder_los
 * \ingroup repr
 *
 * \brief A line of sight for builder robots, which computes the list 3D blocks
 * present in the LOS on request. Note that this is a 2D LOS, even though
 * building 3D structures is 3D (obviously); this is in keeping with making the
 * robots as simple as possible.
 *
 * The line of sight itself is meant to be a read-only view of part of the
 * arena, but it also exposes non-const access to the blocks within that part of
 * the arena by necessity for event processing.
 */
class builder_los final : public rer::client<builder_los>,
                          public crepr::graph3D_los<los_spec> {
 public:
  builder_los(const rtypes::type_uuid& c_id,
              graph_view_type&& the_view,
              const rtypes::spatial_dist& c_unit);

  /* not copy constructible or copyo assignable by default */
  builder_los(const builder_los&) = delete;
  builder_los& operator=(const builder_los&) = delete;
};

NS_END(repr, prism);

#endif /* INCLUDE_PRISM_REPR_BUILDER_LOS_HPP_ */
