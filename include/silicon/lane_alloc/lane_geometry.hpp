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
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector3.hpp"

#include "silicon/controller/perception/ct_skel_info.hpp"
#include "silicon/silicon.hpp"

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
                const ssrepr::ct_coord& ingress_cell,
                const ssrepr::ct_coord& egress_cell);

  lane_geometry(const lane_geometry&) = default;
  lane_geometry& operator=(const lane_geometry&) = delete;

  lane_geometry(lane_geometry&&) = default;
  lane_geometry& operator=(lane_geometry&&) = delete;

  const ssrepr::ct_coord& ingress_virt(void) const { return mc_ingress_virt; }
  const ssrepr::ct_coord& egress_virt(void) const { return mc_egress_virt; }

  /**
   * \brief Range in X INCLUDING virtual cells if the lane is parallel to the
   * X-axis; lane width otherwise.
   */
  const rmath::rangez& xrange(void) const { return m_xrange; }

  /**
   * \brief Range in Y INCLUDING virtual cells if the lane is parallel to the
   * Y-axis; lane width otherwise.
   */
  const rmath::rangez& yrange(void) const { return m_yrange; }

  /**
   * \brief Start point of the ingress lane, which is centered in the CT cell at
   * the edge of the target.
   */
  const rmath::vector3d& ingress_pt(void) const { return m_ingress_pt; }

  /**
   * \brief Start point of the egress lane, which is centered in the CT cell at
   * the edge of the target.
   */
  const rmath::vector3d& egress_pt(void) const { return m_egress_pt; }

 private:
  /* clang-format off */
  const ssrepr::ct_coord  mc_ingress_virt;
  const ssrepr::ct_coord  mc_egress_virt;
  const rmath::vector3d mc_cell_corr;

  rmath::vector2z       m_dims2D{};
  rmath::rangez         m_xrange{};
  rmath::rangez         m_yrange{};
  rmath::vector3d       m_ingress_pt{};
  rmath::vector3d       m_egress_pt{};
  /* clang-format on */
};

NS_END(lane_alloc, silicon);

#endif /* INCLUDE_SILICON_LANE_ALLOC_LANE_GEOMETRY_HPP_ */
