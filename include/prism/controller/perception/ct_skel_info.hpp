/**
 * \file ct_skel_info.hpp
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

#ifndef INCLUDE_PRISM_CONTROLLER_PERCEPTION_CT_SKEL_INFO_HPP_
#define INCLUDE_PRISM_CONTROLLER_PERCEPTION_CT_SKEL_INFO_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/types/type_uuid.hpp"
#include "rcppsw/types/spatial_dist.hpp"

#include "prism/prism.hpp"
#include "prism/gmt/repr/ct_coord.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace prism::gmt {
class spc_gmt;
} /* namespace prism::gmt */

NS_START(prism, controller, perception);

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
  explicit ct_skel_info(const pgmt::spc_gmt* target)
      : m_target(target) {}

  size_t vshell_sized(void) const;
  rtypes::spatial_dist vshell_sizer(void) const;

  RCPPSW_WRAP_DECL(rmath::vector3d, roriginr, const);
  RCPPSW_WRAP_DECL(rmath::vector3z, rorigind, const);
  RCPPSW_WRAP_DECL(rmath::vector3d, voriginr, const);
  RCPPSW_WRAP_DECL(rmath::vector3z, vorigind, const);
  RCPPSW_WRAP_DECL(rtypes::spatial_dist, block_unit_dim, const);
  RCPPSW_WRAP_DECL(size_t, unit_dim_factor, const);
  RCPPSW_WRAP_DECL(const rtypes::type_uuid&, id, const);
  RCPPSW_WRAP_DECL(const rmath::radians&, orientation, const);

  rmath::vector3d bbr(bool include_virtual) const;
  rmath::vector3z bbd(bool include_virtual) const;
  rmath::ranged xrspan(bool include_virtual) const;
  rmath::ranged yrspan(bool include_virtual) const;
  const rtypes::discretize_ratio& grid_resolution(void) const;

  size_t n_lanes(void) const;

  rmath::vector3d anchor_loc_abs(const pgrepr::ct_coord& anchor) const;

  pgrepr::ct_coord to_vcoord(const rmath::vector3d& arena_pos) const;
  pgrepr::ct_coord to_vcoord2D(const rmath::vector2d& arena_pos) const;
  pgrepr::ct_coord as_vcoord(const rmath::vector3z& coord) const;
  pgrepr::ct_coord as_rcoord(const rmath::vector3z& coord) const;
  pgrepr::ct_coord to_rcoord(const rmath::vector3d& arena_pos) const;
  pgrepr::ct_coord to_rcoord2D(const rmath::vector2d& arena_pos) const;

 private:
  /* clang-format off */
  /**
   * This is a reference to an ACTUAL target, not a clone owned by the robot, so
   * it MUST be read-only, and only used to get bare-bones info about the
   * target, otherwise this class will act as more of an oracle.
   */
  const pgmt::spc_gmt* m_target;
  /* clang-format on */
};

NS_END(perception, controller, prism);

#endif /* INCLUDE_PRISM_CONTROLLER_PERCEPTION_CT_SKEL_INFO_HPP_ */
