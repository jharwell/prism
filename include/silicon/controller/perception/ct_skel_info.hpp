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

#include "silicon/silicon.hpp"

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

  rmath::vector3d center(void) const;
  rmath::vector3d originr(void) const;
  rmath::vector3z origind(void) const;
  rmath::ranged xrange(void) const;
  rmath::ranged yrange(void) const;
  rmath::vector3d bbr(void) const;
  rmath::vector3z bbd(void) const;
  double block_unit_dim(void) const;
  size_t unit_dim_factor(void) const;
  const rmath::radians& orientation(void) const;
  size_t n_lanes(void) const;

  rmath::vector3d cell_loc_abs(const rmath::vector3z& coord) const;
  rtypes::type_uuid id(void) const;

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
