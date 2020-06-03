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

size_t ct_skel_info::vshell_sized(void) const{
  return m_target->vshell_sized();
} /* vshell_sized() */

rmath::vector3d ct_skel_info::center(void) const {
  return m_target->voriginr() + rmath::vector3d(m_target->xrsize() / 2.0,
                                               m_target->yrsize() / 2.0,
                                               m_target->zrsize() / 2.0);
} /* center() */

rmath::vector3d ct_skel_info::bbr(void) const {
  /*
   * Targets are surrounded by layers of virtual cells, which we don't care
   * about when allocating construction lanes.
   */
  return {m_target->xrsize() - m_target->vshell_sizer().v() * 2,
        m_target->yrsize() - m_target->vshell_sizer().v() * 2,
        m_target->zrsize()};
} /* bbr() */

rmath::vector3z ct_skel_info::bbd(bool include_virtual) const {
  /*
   * Targets are surrounded by layers of virtual cells, which we don't care
   * about when allocating construction lanes.
   */
  return {m_target->xdsize() - m_target->vshell_sized() * 2 * !include_virtual,
        m_target->ydsize() - m_target->vshell_sized() * 2 * !include_virtual,
        m_target->zdsize()};
} /* bbd() */

const rmath::radians& ct_skel_info::orientation(void) const {
  return m_target->orientation();
} /* orientation() */

rmath::vector3d ct_skel_info::cell_loc_abs(const rmath::vector3z& coord) const {
  return m_target->cell_loc_abs(m_target->access(coord));
} /* cell_loc_abs() */

rtypes::type_uuid ct_skel_info::id(void) const {
  return m_target->id();
} /* id() */

rmath::ranged ct_skel_info::xrange(void) const {
  return m_target->xranger(true);
} /* xrange() */

rmath::ranged ct_skel_info::yrange(void) const {
  return m_target->yranger(true);
} /* yrange() */

double ct_skel_info::block_unit_dim(void) const {
  return m_target->block_unit_dim();
} /* block_unit_dim() */

size_t ct_skel_info::unit_dim_factor(void) const {
  return m_target->unit_dim_factor();
} /* unit_dim_factor() */

NS_END(perception, controller, silicon);
