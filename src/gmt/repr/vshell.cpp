/**
 * \file vshell.cpp
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
#include "prism/gmt/repr/vshell.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, repr);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
vshell::vshell(const carena::ds::arena_grid* grid,
               const rmath::vector3d& ct_origin,
               const rmath::vector3z& ct_bb,
               const rtypes::spatial_dist& block_unit_dim)
    : ER_CLIENT_INIT("prism.gmt.vshell"),
      mc_unit_dim(block_unit_dim),
      /*
       * Subtract padding for the layers of virtual cells surrounding the
       * structure so the origin (LL corner) of the real structure is as
       * configured in the input file.
       *
       * Add padding for the two layers of virtual cells surrounding the
       * structure to get the UR.
       */
      m_outer(outer_init(grid, ct_origin, ct_bb)),
      m_inner(inner_init(grid, ct_origin, ct_bb)) {
  ER_ASSERT(initialization_checks(), "Initialization checks failed");
}

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/
vshell::grid_view_entity_type vshell::outer_init(
    const carena::ds::arena_grid* grid,
    const rmath::vector3d& ct_origin,
    const rmath::vector3z& ct_bb) const {
  auto shell_dvec = rmath::vector2d(vshell::sh_sizer().v(),
                                    vshell::sh_sizer().v());
  auto shell_zvec = rmath::vector2z(vshell::sh_sized(),
                                    vshell::sh_sized());
  auto real_bb = rmath::zvec2dvec(ct_bb, grid->resolution().v());
  auto ll = rmath::dvec2zvec(ct_origin.to_2D(), grid->resolution().v()) - shell_zvec;
  auto ur = rmath::dvec2zvec(ct_origin.to_2D(), grid->resolution().v()) +
            ct_bb.to_2D() + shell_zvec;
  return grid_view_entity_type(rtypes::type_uuid{-1}, /* unused for now */
                               grid->layer<cads::arena_grid::kCell>()->subgrid(ll, ur),
                                   ct_bb.z(),

                                   grid->resolution());
} /* outer_init() */

vshell::grid_view_entity_type vshell::inner_init(
    const carena::ds::arena_grid* grid,
    const rmath::vector3d& ct_origin,
    const rmath::vector3z& ct_bb) const {
  auto shell_dvec = rmath::vector2d(vshell::sh_sizer().v(),
                                    vshell::sh_sizer().v());
  auto shell_zvec = rmath::vector2z(vshell::sh_sized(),
                                    vshell::sh_sized());
  auto real_bb = rmath::zvec2dvec(ct_bb, grid->resolution().v());
  auto ll = rmath::dvec2zvec(ct_origin.to_2D(), grid->resolution().v());
  auto ur = rmath::dvec2zvec(ct_origin.to_2D(), grid->resolution().v()) + ct_bb.to_2D();
  return grid_view_entity_type(rtypes::type_uuid{-1}, /* unused for now */
                               grid->layer<cads::arena_grid::kCell>()->subgrid(ll, ur),                               ct_bb.z(),
                               grid->resolution());
} /* inner_init() */

bool vshell::initialization_checks(void) const {
  ER_CHECK(rmath::is_multiple_of(voriginr().x(), m_outer.resolution().v()) &&
           rmath::is_multiple_of(voriginr().y(), m_outer.resolution().v()),
           "Virtual origin %s (X,Y) coordinates not a multiple of 3D grid resolution %f",
           rcppsw::to_string(voriginr()).c_str(),
           m_outer.resolution().v());

  ER_CHECK(rmath::is_multiple_of(roriginr().x(), m_outer.resolution().v()) &&
           rmath::is_multiple_of(roriginr().y(), m_outer.resolution().v()),
           "Real origin %s (X,Y) coordinates not a multiple of 3D grid resolution %f",
           rcppsw::to_string(roriginr()).c_str(),
           m_outer.resolution().v());

  return true;

error:
  return false;
} /* initialization_checks() */

NS_END(repr, gmt, prism);
