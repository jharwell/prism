/**
 * \file builder_los.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
 *****************************************************************************/
#include "silicon/repr/builder_los.hpp"

#include "cosm/ds/cell3D.hpp"

#include "silicon/repr/construction_lane.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, repr);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
builder_los::builder_los(const rtypes::type_uuid& c_id,
                         const grid_view_type& c_view,
                         const rtypes::discretize_ratio& c_resolution)
    : losQ3D(c_id, c_view, c_resolution),
      ER_CLIENT_INIT("silicon.repr.builder_los") {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
cds::block3D_vectorno builder_los::blocks(void) const {
  cds::block3D_vectorno blocks{};
  for (size_t i = 0; i < xdsize(); ++i) {
    for (size_t j = 0; j < ydsize(); ++j) {
      auto& cell = access(i, j, 0);
      if (cell.state_has_block()) {
        ER_ASSERT(nullptr != cell.block(),
                  "Cell at(%zu,%zu) in HAS_BLOCK state, but does not have block",
                  i,
                  j);
        blocks.push_back(cell.block());
      }
    } /* for(j..) */
  } /* for(i..) */
  return blocks;
} /* blocks() */

NS_END(repr, silicon);
