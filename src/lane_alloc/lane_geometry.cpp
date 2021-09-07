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
#include "silicon/structure/utils.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, lane_alloc);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
lane_geometry::lane_geometry(const scperception::ct_skel_info* target,
                             const ssrepr::ct_coord& ingress_virt,
                             const ssrepr::ct_coord& egress_virt)
    : ER_CLIENT_INIT("silicon.lane_alloc.lane_geometry"),
      mc_ingress_virt{ ingress_virt.to_virtual() },
      mc_egress_virt{ egress_virt.to_virtual() },
      /*
       * Need to account for block width so we calculate the CENTER of the
       * ingress/egress lanes, because the \ref ct->cell_loc_abs() function
       * returns the real coordinates of the LL corner of the cell, per
       * convention.
       */
      mc_cell_corr{ target->block_unit_dim().v() / 2.0,
                    target->block_unit_dim().v() / 2.0,
                    0.0 } {
        ER_ASSERT(sstructure::orientation_valid(target->orientation()),
                  "Bad orientation: '%s'",
                  rcppsw::to_string(target->orientation()).c_str());

        auto l1 = rmath::l1norm(mc_egress_virt.offset(),
                                mc_ingress_virt.offset());
        if (rmath::radians::kZERO == target->orientation()) {
          m_ingress_pt = target->anchor_loc_abs(mc_ingress_virt) + mc_cell_corr;
          m_egress_pt = target->anchor_loc_abs(mc_egress_virt) + mc_cell_corr;

          m_xrange = { 0, target->bbd(true).x() };
          m_yrange = { mc_egress_virt.offset().y(), mc_egress_virt.offset().y() + l1 };

        } else if (rmath::radians::kPI_OVER_TWO == target->orientation()) {
          m_ingress_pt = target->anchor_loc_abs(mc_ingress_virt) + mc_cell_corr;
          m_egress_pt = target->anchor_loc_abs(mc_egress_virt) + mc_cell_corr;

          m_xrange = { mc_ingress_virt.offset().x(),
                       mc_ingress_virt.offset().x() + l1 };
          m_yrange = { 0, target->bbd(true).y() };
        } else if (rmath::radians::kPI == target->orientation()) {
          m_ingress_pt = target->anchor_loc_abs(mc_ingress_virt) + mc_cell_corr;
          m_egress_pt = target->anchor_loc_abs(mc_egress_virt) + mc_cell_corr;

          m_xrange = { 0, target->bbd(true).x() };
          m_yrange = { mc_ingress_virt.offset().y(),
                       mc_ingress_virt.offset().y() + l1 };
        } else if (rmath::radians::kTHREE_PI_OVER_TWO == target->orientation()) {
          m_ingress_pt = target->anchor_loc_abs(mc_ingress_virt) + mc_cell_corr;
          m_egress_pt = target->anchor_loc_abs(mc_egress_virt) + mc_cell_corr;

          m_xrange = { mc_egress_virt.offset().x(), mc_egress_virt.offset().x() + l1 };
          m_yrange = { 0, target->bbd(true).y() };
        }
      }

NS_END(lane_alloc, silicon);
