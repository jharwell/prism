/**
 * \file ct_coord.cpp
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
#include "prism/gmt/repr/ct_coord.hpp"

#include "prism/controller/perception/ct_skel_info.hpp"
#include "prism/gmt/spc_gmt.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, repr);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
ct_coord::ct_coord(const rmath::vector3z& offset,
                   const relativity& relative_to,
                   const spc_gmt* ct)
    : ER_CLIENT_INIT("prism.gmt.ct_coord"),
      mc_ct(ct),
      m_offset(offset),
      m_relative_to(relative_to) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
ct_coord ct_coord::to_virtual(const ct_coord& coord, const spc_gmt* ct) {
  auto offset = coord.offset();
  if (relativity::ekRORIGIN == coord.relative_to()) {
    offset = (ct->rorigind() - ct->vorigind()) + coord.offset();
  }

  return { offset, relativity::ekVORIGIN, ct };
} /* to_virtual() */

ct_coord ct_coord::to_real(const ct_coord& coord, const spc_gmt* ct) {
  auto offset = coord.offset();
  if (relativity::ekVORIGIN == coord.relative_to()) {
    offset = coord.offset() - (ct->rorigind() - ct->vorigind());
  }

  return { offset, relativity::ekRORIGIN, ct };
} /* to_real() */

ct_coord ct_coord::to_real(void) const {
  ER_ASSERT(relativity::ekUNDEFINED != relative_to(),
            "CT coordinates %s undefined!",
            rcppsw::to_string(offset()).c_str());

  return ct_coord::to_real(*this, mc_ct);
}

ct_coord ct_coord::to_virtual(void) const {
  ER_ASSERT(relativity::ekUNDEFINED != relative_to(),
            "CT coordinates %s undefined!",
            rcppsw::to_string(offset()).c_str());
  return ct_coord::to_virtual(*this, mc_ct);
}

ct_coord ct_coord::from_arena(rmath::vector3d pos, const spc_gmt* ct) {
  /*
   * If the unit dim factor is > 1, it is possible that due to floating point
   * errors and the fact that robots traverse the middle of the ingress/egress
   * lane that our cell location within the construction target is not correct,
   * so we correct it here.
   */
  /* coord.x(coord.x() - */
  /*         ct->unit_dim_factor() * ((coord.x() % ct->unit_dim_factor()) != 0)); */
  /* coord.y(coord.y() - */
  /*         ct->unit_dim_factor() * ((coord.y() % ct->unit_dim_factor()) != 0)); */

  auto arena_cell = rmath::dvec2zvec(pos - ct->voriginr(),
                                     ct->arena_grid_resolution().v());
  return from_arena(arena_cell, ct);
} /* from_arena() */

ct_coord ct_coord::from_arena(rmath::vector3z arena_cell, const spc_gmt* ct) {
  auto ct_cell = arena_cell / ct->unit_dim_factor();
  return ct_coord{ ct_cell, relativity::ekVORIGIN, ct };
} /* from_arena() */

std::string ct_coord::to_str(void) const {
  return rcppsw::to_string(m_offset) +
         ((relativity::ekVORIGIN == m_relative_to)
              ? "@VORIGIN"
              : (relativity::ekRORIGIN == m_relative_to) ? "@RORIGIN"
                                                         : "@UNDEFINED");
} /* to_str() */

NS_END(repr, gmt, prism);
