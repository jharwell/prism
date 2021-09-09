/**
 * \file slice2D.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/gmt/repr/slice2D.hpp"

#include <algorithm>

#include "prism/gmt/utils.hpp"
#include "prism/gmt/repr/vshell.hpp"
#include "prism/gmt/ds/connectivity_graph.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, repr);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
slice2D::slice2D(const slice2D_spec& spec, const pgds::connectivity_graph* graph)
    : ER_CLIENT_INIT("prism.gmt.slice2D"),
      /*
       * @bug this assumes that the calculated center of the subgraph graph is
       * always an anchor node, which is not true.
       */
      graph3D_view_entity(rtypes::constants::kNoUUID, /* not needed for now */
                          graph->subgraph(*graph->decoratee().find(spec.center()),
                                          spec.radius(),
                                          boost::make_optional(spec.axis())),
                          spec.unit_dim()),
      mc_spec(spec) {
  ER_DEBUG("Spec: %s,n_vertices=%zu",
           rcppsw::to_string(mc_spec).c_str(),
           n_vertices());

  ER_DEBUG("rcenter3D=%s,dcenter3D=%s",
           rcppsw::to_string(rcenter3D()).c_str(),
           rcppsw::to_string(dcenter3D()).c_str());

  ER_DEBUG("xrspan=%s,yrspan=%s,zrspan=%s,xdspan=%s,ydspan=%s,zdspan=%s",
           rcppsw::to_string(xrspan()).c_str(),
           rcppsw::to_string(yrspan()).c_str(),
           rcppsw::to_string(zrspan()).c_str(),
           rcppsw::to_string(xdspan()).c_str(),
           rcppsw::to_string(ydspan()).c_str(),
           rcppsw::to_string(zdspan()).c_str());
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool slice2D::cells_are_adjacent(const rmath::vector3z& cell1,
                                 const rmath::vector3z& cell2) const {
  ER_ASSERT(contains(cell1) && contains(cell2),
            "Slice %s does not contain %s and %s",
            rcppsw::to_string(mc_spec).c_str(),
            rcppsw::to_string(cell1).c_str(),
            rcppsw::to_string(cell2).c_str());

  bool d1plus1_neighbor, d1minus1_neighbor, d2plus1_neighbor, d2minus1_neighbor;
  if (rmath::vector3z::Z == mc_spec.axis()) {
    d1plus1_neighbor = (cell1 + rmath::vector3z::X == cell2);
    d1minus1_neighbor = (cell1 - rmath::vector3z::X == cell2);
    d2plus1_neighbor = (cell1 + rmath::vector3z::Y == cell2);
    d2minus1_neighbor = (cell1 - rmath::vector3z::Y == cell2);
  } else if (rmath::vector3z::Y == mc_spec.axis()) {
    d1plus1_neighbor = (cell1 + rmath::vector3z::X == cell2);
    d1minus1_neighbor = (cell1 - rmath::vector3z::X == cell2);
    d2plus1_neighbor = (cell1 + rmath::vector3z::Z == cell2);
    d2minus1_neighbor = (cell1 - rmath::vector3z::Z == cell2);
  } else {
    d1plus1_neighbor = (cell1 + rmath::vector3z::Y == cell2);
    d1minus1_neighbor = (cell1 - rmath::vector3z::Y == cell2);
    d2plus1_neighbor = (cell1 + rmath::vector3z::Z == cell2);
    d2minus1_neighbor = (cell1 - rmath::vector3z::Z == cell2);
  }

  return d1plus1_neighbor || d1minus1_neighbor || d2plus1_neighbor ||
         d2minus1_neighbor;
} /* cells_are_adjacent() */

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
slice2D_spec slice2D::spec_calc(const rmath::vector3z& axis,
                                size_t axis_offset,
                                const pgrepr::vshell* vshell) {
  rmath::vector3z center;
  rtypes::manhattan_dist radius(std::max({vshell->real()->xdsize(),
                                          vshell->real()->ydsize(),
                                          vshell->real()->zdsize()}));

  /*
   * Exactly ONE of these terms will be non-zero on a given invocation:
   * whichever one corresponds to the slice axis.
   */
  center = axis.mask(rmath::vector3z::X) * axis_offset +
           axis.mask(rmath::vector3z::Y) * axis_offset +
           axis.mask(rmath::vector3z::Z) * axis_offset;


  /*
   * Exactly TWO of these terms will be non-zero on a given invocation:
   * whichever two were not the slice axis.
   */
  center += rmath::vector3z::X * (vshell->real()->xdsize() / 2 *
                                  (axis.mask(rmath::vector3z::X).length() <= 0));
  center += rmath::vector3z::Y * (vshell->real()->ydsize() / 2 *
                                  (axis.mask(rmath::vector3z::Y).length() <= 0));
  center += rmath::vector3z::Z * (vshell->real()->zdsize() / 2 *
                                  (axis.mask(rmath::vector3z::Z).length() <= 0));

  return { axis, center, radius, vshell->unit_dim()};
} /* spec_calc() */

NS_END(repr, gmt, prism);
