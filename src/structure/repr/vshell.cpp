/**
 * \file vshell.cpp
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
#include "silicon/structure/repr/vshell.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, repr);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
vshell::vshell(const carena::ds::arena_grid* grid,
               const rmath::vector3d& ct_origin,
               const rmath::vector3z& ct_dims,
               const rtypes::spatial_dist& block_unit_dim)
    : ER_CLIENT_INIT("silicon.structure.vshell"),
      mc_unit_dim(block_unit_dim),
      /*
       * Subtract padding for the layers of virtual cells surrounding the
       * structure so the origin (LL corner) of the real structure is as
       * configured in the input file.
       *
       * Add padding for the two layers of virtual cells surrounding the
       * structure to get the UR.
       */
      m_outer(rtypes::type_uuid{-1}, /* unused for now */
              grid->layer<cads::arena_grid::kCell>()->subgrid(
                  rmath::dvec2zvec(ct_origin.to_2D() - rmath::vector2d(sh_sizer().v(),
                                                                       sh_sizer().v()),
                                   grid->resolution().v()),
                  rmath::dvec2zvec(ct_origin.to_2D() +
                                   rmath::vector2d(sh_sizer().v(), sh_sizer().v()) * 2,
                                   grid->resolution().v()) +
                  ct_dims.to_2D()),
              grid->resolution()),
      m_inner(rtypes::type_uuid{-1}, /* unused for now */
              grid->layer<cads::arena_grid::kCell>()->subgrid(
                  rmath::dvec2zvec(ct_origin.to_2D(), grid->resolution().v()),
                  rmath::dvec2zvec(ct_origin.to_2D(),
                                   grid->resolution().v()) + ct_dims.to_2D()),
              grid->resolution()) {
  ER_ASSERT(initialization_checks(), "Initialization checks failed");
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
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

NS_END(repr, structure, silicon);
