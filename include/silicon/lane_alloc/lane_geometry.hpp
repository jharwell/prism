/**
 * \file lane_geometry.hpp
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

#ifndef INCLUDE_SILICON_LANE_ALLOC_LANE_GEOMETRY_HPP_
#define INCLUDE_SILICON_LANE_ALLOC_LANE_GEOMETRY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/er/client.hpp"

#include "silicon/silicon.hpp"
#include "silicon/controller/perception/ct_skel_info.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, lane_alloc);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class lane_geometry
 * \ingroup lane_alloc
 *
 * \brief Representation of the geometry (ingress/egress points, center, etc) of
 * an allocated construction lane.
 */
class lane_geometry : public rer::client<lane_geometry> {
 public:
  lane_geometry(const scperception::ct_skel_info* target,
                const rmath::vector3z& ingress_nearest_cell,
                const rmath::vector3z& egress_nearest_cell);

  lane_geometry(const lane_geometry&) = default;
  lane_geometry& operator=(const lane_geometry&) = default;

  lane_geometry(lane_geometry&&) = default;
  lane_geometry& operator=(lane_geometry&&) = delete;

  const rmath::vector3z& ingress_cell(void) const {
    return mc_ingress_cell;
  }
  const rmath::vector3z& egress_cell(void) const {
    return mc_egress_cell;
  }

  const rmath::vector3d& ingress_start(void) const { return m_ingress_start; }
  const rmath::vector3d& egress_start(void) const { return m_egress_start; }

  const rmath::vector3d& ingress_center(void) const { return m_ingress_center; }
  const rmath::vector3d& egress_center(void) const { return m_egress_center; }

 private:
  /* clang-format off */
  const rmath::vector3z mc_ingress_cell;
  const rmath::vector3z mc_egress_cell;
  const rmath::vector3d mc_cell_corr;

  rmath::vector3d m_ingress_start{};
  rmath::vector3d m_egress_start{};
  rmath::vector3d m_ingress_center{};
  rmath::vector3d m_egress_center{};
  /* clang-format on */
};

NS_END(lane_alloc, silicon);

#endif /* INCLUDE_SILICON_LANE_ALLOC_LANE_GEOMETRY_HPP_ */
