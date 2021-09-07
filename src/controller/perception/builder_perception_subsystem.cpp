/**
 * \file builder_perception_subsystem.cpp
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
#include "silicon/controller/perception/builder_perception_subsystem.hpp"

#include "rcppsw/math/angles.hpp"

#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

#include "silicon/controller/perception/perception_receptor.hpp"
#include "silicon/fsm/calculators/lane_alignment.hpp"
#include "silicon/repr/construction_lane.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, controller, perception);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
builder_perception_subsystem::builder_perception_subsystem(
    const cspconfig::rlos_config* const config,
    const csubsystem::sensing_subsystemQ3D* const sensing)
    : rlos_perception_subsystem(config),
      mc_arena_res(config->grid2D.resolution),
      mc_arena_xrspan(0.0, config->grid2D.dims.x()),
      mc_arena_yrspan(0.0, config->grid2D.dims.y()),
      mc_sensing(sensing),
      mc_builder_prox(this, mc_sensing),
      m_receptor(nullptr) {}

builder_perception_subsystem::~builder_perception_subsystem(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void builder_perception_subsystem::update() {
  /* Open to extension (I will probably need to put stuff here eventually...) */
} /* update() */

const scperception::ct_skel_info*
builder_perception_subsystem::nearest_ct(void) const {
  return m_receptor->nearest_ct(mc_sensing->rpos3D());
} /* nearest_ct() */

void builder_perception_subsystem::receptor(
    std::unique_ptr<perception_receptor> receptor) {
  m_receptor = std::move(receptor);
} /* receptor() */

bool builder_perception_subsystem::is_behind_self(
    const rmath::vector2d& framed_offset,
    const srepr::construction_lane* lane) const {
  auto self_state = lane_traversal_state(lane, mc_sensing->rpos2D());

  if (rmath::radians::kZERO == lane->orientation()) {
    if (self_state.in_ingress) {
      return framed_offset.x() < 0;
    } else if (self_state.in_egress) {
      return framed_offset.x() > 0;
    }
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation()) {
    if (self_state.in_ingress) {
      return framed_offset.y() < 0;
    } else if (self_state.in_egress) {
      return framed_offset.y() > 0;
    }
  } else if (rmath::radians::kPI == lane->orientation()) {
    if (self_state.in_ingress) {
      return framed_offset.x() > 0;
    } else if (self_state.in_egress) {
      return framed_offset.x() < 0;
    }
  }  else if (rmath::radians::kTHREE_PI_OVER_TWO == lane->orientation()) {
    if (self_state.in_ingress) {
      return framed_offset.y() > 0;
    } else if (self_state.in_egress) {
      return framed_offset.y() < 0;
    }
  }
  return false;
} /* is_behind_self() */

bool builder_perception_subsystem::self_lane_aligned(
    const srepr::construction_lane* lane) const {
  auto range = rmath::range<rmath::radians>{
    lane->orientation() - sfsm::calculators::lane_alignment::kAZIMUTH_TOL,
    lane->orientation() + sfsm::calculators::lane_alignment::kAZIMUTH_TOL
  };
  auto azimuth = mc_sensing->azimuth();
  /* PI factor because we might be heading AGAINST the lane orientation */
  return range.contains(azimuth.unsigned_normalize()) ||
         range.contains((azimuth + rmath::radians::kPI).unsigned_normalize());
} /* self_lane_aligned() */

compass_orientation
    builder_perception_subsystem::self_compass_orientation(rmath::radians azimuth) const {
  return { is_aligned_with(azimuth, rmath::radians::kZERO),
           is_aligned_with(azimuth, rmath::radians::kPI_OVER_TWO),
           is_aligned_with(azimuth, rmath::radians::kPI),
           is_aligned_with(azimuth, rmath::radians::kTHREE_PI_OVER_TWO) };
} /* self_compass_orientation() */

bool builder_perception_subsystem::is_aligned_with(rmath::radians azimuth,
                                                   const rmath::radians& target) const {
  auto res = rmath::normalized_diff(target, azimuth);
  return rmath::abs(res) <= sfsm::calculators::lane_alignment::kAZIMUTH_TOL;
} /* is_aligned_with() */

lane_traversal_state
builder_perception_subsystem::lane_traversal_state(
    const srepr::construction_lane* lane,
    const rmath::vector2d& rpos) const {
  struct lane_traversal_state ret;

  /*
   * We are not on the structure/have allocated a lane yet, and so by definition
   * are not sharing it with another robot.
   */
  if (nullptr == lane) {
    return ret;
  }
  auto ct = nearest_ct();
  auto self_vcoord2D = ct->to_vcoord2D(rpos);

  auto ingress = lane->geometry().ingress_virt();
  auto egress = lane->geometry().egress_virt();

  if (rmath::radians::kZERO == lane->orientation()) {
    ret.in_ingress = ingress.offset().y() == self_vcoord2D.offset().y();
    ret.in_egress = egress.offset().y() == self_vcoord2D.offset().y();
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation()) {
    ret.in_ingress = ingress.offset().x() == self_vcoord2D.offset().x();
    ret.in_egress = egress.offset().x() == self_vcoord2D.offset().x();
  } else if (rmath::radians::kPI == lane->orientation()) {
    ret.in_ingress = ingress.offset().y() == self_vcoord2D.offset().y();
    ret.in_egress = egress.offset().y() == self_vcoord2D.offset().y();
  }  else if (rmath::radians::kTHREE_PI_OVER_TWO == lane->orientation()) {
    ret.in_ingress = ingress.offset().x() == self_vcoord2D.offset().x();
    ret.in_egress = egress.offset().x() == self_vcoord2D.offset().x();
  }
  return ret;
} /* lane_traversal_state() */

NS_END(perception, controller, silicon);
