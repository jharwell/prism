/**
 * \file lane_geometry.cpp
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
#include "silicon/lane_alloc/lane_geometry.hpp"

#include "silicon/controller/perception/ct_skel_info.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, lane_alloc);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
lane_geometry::lane_geometry(const scperception::ct_skel_info* target,
                             const rmath::vector3z& ingress_nearest_cell,
                             const rmath::vector3z& egress_nearest_cell)
    : ER_CLIENT_INIT("silicon.lane_alloc.lane_geometry"),
      mc_ingress_cell(ingress_nearest_cell),
      mc_egress_cell(egress_nearest_cell),
      /*
       * Need to account for block width so we calculate the CENTER of the
       * ingress/egress lanes, because the \ref ct->cell_loc_abs() function
       * returns the real coordinates of the LL corner of the cell, per
       * convention.
       */
      mc_cell_corr{ target->block_unit_dim() / 2.0,
                    target->block_unit_dim() / 2.0,
                    0.0 } {
  if (rmath::radians::kZERO == target->orientation()) {
    m_ingress_start = target->cell_loc_abs(mc_ingress_cell) + mc_cell_corr;
    m_egress_start = target->cell_loc_abs(mc_egress_cell) + mc_cell_corr;
    m_ingress_center =
        rmath::vector3d(target->xrange().center(), m_ingress_start.y(), 0.0) +
        mc_cell_corr;
    m_egress_center =
        rmath::vector3d(target->xrange().center(), m_egress_start.y(), 0.0) +
        mc_cell_corr;

  } else if (rmath::radians::kPI_OVER_TWO == target->orientation()) {
    m_ingress_start = target->cell_loc_abs(mc_ingress_cell) + mc_cell_corr;
    m_egress_start = target->cell_loc_abs(mc_egress_cell) + mc_cell_corr;
    m_ingress_center =
        rmath::vector3d(m_ingress_start.x(), target->yrange().center(), 0.0) +
        mc_cell_corr;
    m_egress_center =
        rmath::vector3d(m_egress_start.x(), target->yrange().center(), 0.0) +
        mc_cell_corr;

  } else if (rmath::radians::kPI == target->orientation()) {
    m_ingress_start = target->cell_loc_abs(mc_ingress_cell) + mc_cell_corr;
    m_egress_start = target->cell_loc_abs(mc_egress_cell) + mc_cell_corr;
    m_ingress_center =
        rmath::vector3d(target->xrange().center(), m_ingress_start.y(), 0.0) +
        mc_cell_corr;
    m_egress_center =
        rmath::vector3d(target->xrange().center(), m_egress_start.y(), 0.0) +
        mc_cell_corr;

  } else if (rmath::radians::kTHREE_PI_OVER_TWO == target->orientation()) {
    m_ingress_start = target->cell_loc_abs(mc_ingress_cell) + mc_cell_corr;
    m_egress_start = target->cell_loc_abs(mc_egress_cell) + mc_cell_corr;
    m_ingress_center =
        rmath::vector3d(m_ingress_start.x(), target->yrange().center(), 0.0) +
        mc_cell_corr;
    m_egress_center =
        rmath::vector3d(m_egress_start.x(), target->yrange().center(), 0.0) +
        mc_cell_corr;
  } else {
    ER_FATAL_SENTINEL("Bad lane orientation '%s'",
                      rcppsw::to_string(target->orientation()).c_str());
  }
}

NS_END(lane_alloc, silicon);
