/**
 * \file construction_lane.cpp
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
#include "silicon/repr/construction_lane.hpp"

#include "cosm/ds/cell3D.hpp"

#include "silicon/repr/builder_los.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, repr);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
construction_lane::construction_lane(size_t id,
                                     const rmath::radians& orientation,
                                     const lane_alloc::lane_geometry& geometry)
    : ER_CLIENT_INIT("silicon.repr.construction_lane"),
      m_id(id),
      m_orientation(orientation),
      m_geometry(geometry) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool construction_lane::contains(const ssds::ct_coord& coord) const {
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

boost::optional<frontier_coord>
construction_lane::frontier(const scperception::ct_skel_info* ct,
                            const srepr::builder_los* los) const {
  auto ingress_rel = (geometry().ingress_virt().offset() - los->abs_ll()).to_2D();
  auto egress_rel = (geometry().egress_virt().offset() - los->abs_ll()).to_2D();

  rmath::vector3z ingress_fill, egress_fill;

  bool have_frontier = false;
  if (rmath::radians::kZERO == orientation()) {
    ingress_fill = los->access(0, ingress_rel.y()).loc();
    egress_fill = los->access(0, egress_rel.y()).loc();

    for (size_t i = 0; i < los->xsize(); ++i) {
      if (los->access(i, ingress_rel.y()).state_has_block()) {
        ingress_fill += rmath::vector3z::X;
        have_frontier = true;
      }
      if (los->access(i, egress_rel.y()).state_has_block()) {
        egress_fill += rmath::vector3z::X;
        have_frontier = true;
      }
    } /* for(i..) */
  } else if (rmath::radians::kPI_OVER_TWO == orientation()) {
    ingress_fill = los->access(ingress_rel.x(), 0).loc();
    egress_fill = los->access(egress_rel.x(), 0).loc();

    for (size_t j = 0; j < los->ysize(); ++j) {
      if (los->access(ingress_rel.x(), j).state_has_block()) {
        ingress_fill += rmath::vector3z::Y;
        have_frontier = true;
      }
      if (los->access(egress_rel.x(), j).state_has_block()) {
        egress_fill += rmath::vector3z::Y;
        have_frontier = true;
      }
    } /* for(i..) */
  } else if (rmath::radians::kPI == orientation()) {
    ingress_fill = los->access(0, ingress_rel.y()).loc();
    egress_fill = los->access(0, egress_rel.y()).loc();

    for (size_t i = 0; i < los->xsize(); ++i) {
      if (los->access(i, ingress_rel.y()).state_has_block()) {
        ingress_fill -= rmath::vector3z::X;
        have_frontier = true;
      }
      if (los->access(i, egress_rel.y()).state_has_block()) {
        egress_fill -= rmath::vector3z::X;
        have_frontier = true;
      }
    } /* for(i..) */
  } else if (rmath::radians::kTHREE_PI_OVER_TWO == orientation()) {
    ingress_fill = los->access(ingress_rel.x(), 0).loc();
    egress_fill = los->access(egress_rel.x(), 0).loc();

    for (size_t j = 0; j < los->ysize(); ++j) {
      if (los->access(ingress_rel.x(), j).state_has_block()) {
        ingress_fill -= rmath::vector3z::Y;
        have_frontier = true;
      }
      if (los->access(egress_rel.x(), j).state_has_block()) {
        egress_fill -= rmath::vector3z::Y;
        have_frontier = true;
      }
    } /* for(i..) */
  }

  if (have_frontier) {
    return boost::make_optional(frontier_coord{ ct->as_vcoord(ingress_fill),
                                                ct->as_vcoord(egress_fill) });
  } else {
    return boost::none;
  }
} /* frontier() */

NS_END(repr, silicon);
