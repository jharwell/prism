/**
 * \file ct_skel_info.cpp
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
#include "silicon/controller/perception/ct_skel_info.hpp"

#include "silicon/structure/structure3D.hpp"
#include "silicon/structure/repr/vshell.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, controller, perception);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::vector3d ct_skel_info::roriginr(void) const {
  return m_target->roriginr();
} /* roriginr() */

rmath::vector3z ct_skel_info::rorigind(void) const {
  return m_target->rorigind();
} /* rorigind() */

rmath::vector3d ct_skel_info::voriginr(void) const {
  return m_target->voriginr();
} /* voriginr() */

rmath::vector3z ct_skel_info::vorigind(void) const {
  return m_target->vorigind();
} /* vorigind() */

size_t ct_skel_info::vshell_sized(void) const {
  return m_target->vshell()->sh_sized();
} /* vshell_sized() */

rtypes::spatial_dist ct_skel_info::vshell_sizer(void) const {
  return m_target->vshell()->sh_sizer();
} /* vshell_sizer() */

rmath::vector3d ct_skel_info::center(void) const {
  return m_target->rcenter3D();
} /* center() */

rmath::vector3d ct_skel_info::bbr(bool include_virtual) const {
  return { m_target->vshell()->xrspan(include_virtual).span(),
           m_target->vshell()->yrspan(include_virtual).span(),
           m_target->vshell()->zrspan().span() };
} /* bbr() */

rmath::vector3z ct_skel_info::bbd(bool include_virtual) const {
  return { m_target->vshell()->xdspan(include_virtual).span(),
           m_target->vshell()->ydspan(include_virtual).span(),
           m_target->vshell()->zdspan().span() };
} /* bbd() */

ssds::ct_coord ct_skel_info::to_vcoord(const rmath::vector3d& arena_pos) const {
  auto raw = ssds::ct_coord::from_arena(arena_pos, m_target);
  return raw.to_virtual();
} /* to_vcoord() */

ssds::ct_coord ct_skel_info::as_vcoord(const rmath::vector3z& coord) const {
  return { coord, ssds::ct_coord::relativity::ekVORIGIN, m_target };
} /* to_vcoord() */

ssds::ct_coord ct_skel_info::as_rcoord(const rmath::vector3z& coord) const {
  return { coord, ssds::ct_coord::relativity::ekRORIGIN, m_target };
} /* to_vcoord() */

ssds::ct_coord
ct_skel_info::to_vcoord2D(const rmath::vector2d& arena_coord) const {
  return to_vcoord(rmath::vector3d(arena_coord.x(), arena_coord.y(), 0));
} /* to_vcoord() */

ssds::ct_coord ct_skel_info::to_rcoord(const rmath::vector3d& arena_pos) const {
  auto raw = ssds::ct_coord::from_arena(arena_pos, m_target);
  return raw.to_real();
} /* to_rcoord() */

ssds::ct_coord
ct_skel_info::to_rcoord2D(const rmath::vector2d& arena_pos) const {
  return to_rcoord(rmath::vector3d(arena_pos.x(), arena_pos.y(), 0));
} /* to_rcoord() */

const rmath::radians& ct_skel_info::orientation(void) const {
  return m_target->orientation();
} /* orientation() */

rmath::vector3d
ct_skel_info::anchor_loc_abs(const ssds::ct_coord& anchor) const {
  return m_target->anchor_loc_abs(anchor);
} /* anchor_loc_abs() */

rtypes::type_uuid ct_skel_info::id(void) const {
  return m_target->id();
} /* id() */

rmath::ranged ct_skel_info::xrspan(void) const {
  return m_target->vshell()->xrspan(true); /* include virtual */
} /* xrspan() */

rmath::ranged ct_skel_info::yrspan(void) const {
  return m_target->vshell()->yrspan(true); /* include virtual */
} /* yrspan() */

rtypes::spatial_dist ct_skel_info::block_unit_dim(void) const {
  return m_target->block_unit_dim();
} /* block_unit_dim() */

size_t ct_skel_info::unit_dim_factor(void) const {
  return m_target->unit_dim_factor();
} /* unit_dim_factor() */

const rtypes::discretize_ratio& ct_skel_info::grid_resolution(void) const {
  return m_target->vshell()->real()->resolution();
} /* grid_resolution() */

NS_END(perception, controller, silicon);
