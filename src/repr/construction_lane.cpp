/**
 * \file construction_lane.cpp
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
#include "prism/repr/construction_lane.hpp"

#include "prism/repr/builder_los.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, repr);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
construction_lane::construction_lane(size_t id,
                                     const rmath::radians& orientation,
                                     const lane_alloc::lane_geometry& geometry,
                                     lane_alloc::history* history)
    : ER_CLIENT_INIT("prism.repr.construction_lane"),
      m_id(id),
      m_orientation(orientation),
      m_geometry(geometry),
      m_history(history) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool construction_lane::contains(const pgrepr::ct_coord& coord) const {
  /*
   * Even though construction lanes always use real construction target
   * coordinates, need to include the virtual cells at the front of the lane in
   * order to prevent deadlocks as the last blocks are placed in the lane due to
   * robots following too closely.
   */
  auto virt = coord.to_virtual();

  /*
   * Ignore the Z value of the coordinate for the contains test, as construction
   * lanes are conceptualized as silos which extend to the structure height in
   * Z.
   */
  return geometry().xrange().contains(virt.offset().x()) &&
         geometry().yrange().contains(virt.offset().y());
} /* contains() */

NS_END(repr, prism);
