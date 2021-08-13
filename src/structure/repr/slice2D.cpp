/**
 * \file slice2D.cpp
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
 ******************************************************************************/
#include "silicon/structure/repr/slice2D.hpp"

#include <algorithm>

#include "silicon/structure/utils.hpp"
#include "silicon/structure/repr/vshell.hpp"
#include "silicon/structure/ds/spec_graph.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, repr);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
slice2D::slice2D(const slice2D_spec& spec, const ssds::spec_graph* graph)
    : ER_CLIENT_INIT("silicon.structure.slice2D"),
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
  ER_INFO("Spec: %s", rcppsw::to_string(mc_spec).c_str());
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool slice2D::contains(const rmath::vector3z& c) const {
  return boost::none != decoratee().find(c);
} /* contains() */

size_t slice2D::d1(void) const {
  if (rmath::vector3z::X == mc_spec.axis()) {
    /* d1=y, d2=z */
    return view().ur().y() - view().ll().y();
  } else if (rmath::vector3z::Y == mc_spec.axis()) {
    /* d1=x, d2=z */
    return view().ur().x() - view().ll().x();
  } else {
    /* d1=x, d2=y */
    return view().ur().x() - view().ll().x();
  }
} /* d1() */

size_t slice2D::d2(void) const {
if (rmath::vector3z::X == mc_spec.axis()) {
    /* d1=y, d2=z */
    return view().ur().z() - view().ll().z();
  } else if (rmath::vector3z::Y == mc_spec.axis()) {
    /* d1=x, d2=z */
    return view().ur().z() - view().ll().z();
  } else {
    /* d1=x, d2=y */
    return view().ur().y() - view().ll().y();
  }
} /* d2() */

const cds::cell3D& slice2D::access(size_t d1, size_t d2) const {
  if (rmath::vector3z::X == mc_coords.axis) {
    return mc_view[0][d1][d2];
  } else if (rmath::vector3z::Y == mc_coords.axis) {
    return mc_view[d1][0][d2];
  } else {
    return mc_view[d1][d2][0];
  }
} /* access() */

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
slice2D_spec slice2D::spec_calc(const rmath::vector3z& axis,
                                size_t axis_offset,
                                const ssrepr::vshell* vshell) {
  rmath::vector3z center;
  size_t radius = std::max({vshell->real()->xdsize(),
                            vshell->real()->ydsize(),
                            vshell->real()->zdsize()});
  center = rmath::vector3z(vshell->real()->xdsize() / 2,
                           vshell->real()->ydsize() / 2,
                           vshell->real()->zdsize() / 2);

  /* exactly ONE of these terms will be non-zero on a given invocation */
  center += (rmath::vector3z::X && axis) * axis_offset +
            (rmath::vector3z::Y && axis) * axis_offset +
            (rmath::vector3z::Z && axis) * axis_offset;
  return { axis, center, radius, vshell->unit_dim() };
} /* spec_calc() */

NS_END(repr, structure, silicon);
