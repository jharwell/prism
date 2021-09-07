/**
 * \file ct_skel_info.hpp
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

#ifndef INCLUDE_SILICON_CONTROLLER_PERCEPTION_CT_SKEL_INFO_HPP_
#define INCLUDE_SILICON_CONTROLLER_PERCEPTION_CT_SKEL_INFO_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/types/type_uuid.hpp"
#include "rcppsw/types/spatial_dist.hpp"

#include "silicon/silicon.hpp"
#include "silicon/structure/repr/ct_coord.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace silicon::structure {
class structure3D;
} /* namespace silicon::structure */

NS_START(silicon, controller, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct ct_skel_info
 * \ingroup controller perception
 *
 * \brief Basic barebones information about construction targets that some robot
 * controllers use to make task allocation decisions.
 */
class ct_skel_info {
 public:
  explicit ct_skel_info(const sstructure::structure3D* target)
      : m_target(target) {}

  size_t vshell_sized(void) const;
  rtypes::spatial_dist vshell_sizer(void) const;
  rmath::vector3d center(void) const;
  rmath::vector3d roriginr(void) const;
  rmath::vector3z rorigind(void) const;
  rmath::vector3d voriginr(void) const;
  rmath::vector3z vorigind(void) const;
  rmath::ranged xrspan(void) const;
  rmath::ranged yrspan(void) const;
  rmath::vector3d bbr(bool include_virtual = false) const;
  rmath::vector3z bbd(bool include_virtual = false) const;
  rtypes::spatial_dist block_unit_dim(void) const;
  size_t unit_dim_factor(void) const;
  const rmath::radians& orientation(void) const;
  size_t n_lanes(void) const;
  const rtypes::discretize_ratio& grid_resolution(void) const;

  rmath::vector3d anchor_loc_abs(const ssrepr::ct_coord& anchor) const;
  rtypes::type_uuid id(void) const;
  ssrepr::ct_coord to_vcoord(const rmath::vector3d& arena_pos) const;
  ssrepr::ct_coord to_vcoord2D(const rmath::vector2d& arena_pos) const;
  ssrepr::ct_coord as_vcoord(const rmath::vector3z& coord) const;
  ssrepr::ct_coord as_rcoord(const rmath::vector3z& coord) const;
  ssrepr::ct_coord to_rcoord(const rmath::vector3d& arena_pos) const;
  ssrepr::ct_coord to_rcoord2D(const rmath::vector2d& arena_pos) const;

 private:
  /* clang-format off */
  /**
   * This is a reference to an ACTUAL target, not a clone owned by the robot, so
   * it MUST be read-only, and only used to get bare-bones info about the
   * target, otherwise this class will act as more of an oracle.
   */
  const sstructure::structure3D* m_target;
  /* clang-format on */
};

NS_END(perception, controller, silicon);

#endif /* INCLUDE_SILICON_CONTROLLER_PERCEPTION_CT_SKEL_INFO_HPP_ */
