/**
 * \file builder_util_fsm.cpp
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
#include "silicon/fsm/builder_util_fsm.hpp"

#include "rcppsw/patterns/fsm/event.hpp"

#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"

#include "silicon/controller/perception/builder_perception_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, fsm);

/*******************************************************************************
 * Class Constants
 ******************************************************************************/
const rmath::radians builder_util_fsm::kROBOT_AZIMUTH_TOL = rmath::radians(0.10);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
builder_util_fsm::builder_util_fsm(
    const scperception::builder_perception_subsystem* perception,
    crfootbot::footbot_saa_subsystem* saa,
    rmath::rng* rng,
    uint8_t max_states)
    : util_hfsm(saa, rng, max_states),
      ER_CLIENT_INIT("silicon.fsm.builder_util"),
      HFSM_CONSTRUCT_STATE(wait_for_robot, hfsm::top_state()),
      mc_perception(perception) {}

builder_util_fsm::~builder_util_fsm(void) = default;

/*******************************************************************************
 * States
 ******************************************************************************/
HFSM_STATE_DEFINE(builder_util_fsm, wait_for_robot, const robot_wait_data* data) {
  saa()->actuation()->actuator<ckin2D::governed_diff_drive>()->reset();

  if (robot_proximity_type::ekTRAJECTORY == data->prox_type &&
      !robot_trajectory_proximity()) {
    internal_event(previous_state());
  } else if (robot_proximity_type::ekMANHATTAN == data->prox_type &&
             !robot_manhattan_proximity()) {
    internal_event(previous_state());
  }
  return rpfsm::event_signal::ekHANDLED;
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void builder_util_fsm::init(void) {
  actuation()->reset();
  util_hfsm::init();
} /* init() */

bool builder_util_fsm::lane_alignment_verify_pos(
    const rmath::vector2d& lane_point,
    const rmath::radians& orientation) const {
  auto dist_diff = sensing()->rpos2D() - lane_point;
  if (rmath::radians::kZERO == orientation) {
    ER_CHECK(std::fabs(dist_diff.y()) <= kLANE_VECTOR_DIST_TOL,
             "Robot@%s too far from ingress@%s in Y for lane orientated@'%s': "
             "%f > %f",
             rcppsw::to_string(sensing()->rpos2D()).c_str(),
             rcppsw::to_string(lane_point).c_str(),
             rcppsw::to_string(orientation).c_str(),
             std::fabs(dist_diff.y()),
             kLANE_VECTOR_DIST_TOL);
  } else if (rmath::radians::kPI_OVER_TWO == orientation) {
    ER_CHECK(std::fabs(dist_diff.x()) <= kLANE_VECTOR_DIST_TOL,
             "Robot@%s too far from ingress@%s in X for lane orientated@'%s': "
             "%f > %f",
             rcppsw::to_string(sensing()->rpos2D()).c_str(),
             rcppsw::to_string(lane_point).c_str(),
             rcppsw::to_string(orientation).c_str(),
             std::fabs(dist_diff.x()),
             kLANE_VECTOR_DIST_TOL);
  } else {
    ER_FATAL_SENTINEL("Bad lane orientation '%s'",
                      rcppsw::to_string(orientation).c_str());
  }
  return true;

error:
  return false;
} /* lane_alignment_verify_pos() */

bool builder_util_fsm::lane_alignment_verify_azimuth(
    const rmath::radians& orientation) const {
  rmath::radians angle_diff;
  if (rmath::radians::kZERO == orientation) {
    angle_diff =
        (rmath::radians::kPI - saa()->sensing()->azimuth()).signed_normalize();
  } else if (rmath::radians::kPI_OVER_TWO == orientation) {
    angle_diff =
        (rmath::radians::kTHREE_PI_OVER_TWO - saa()->sensing()->azimuth())
            .signed_normalize();
  } else {
    ER_FATAL_SENTINEL("Bad lane orientation '%s'",
                      rcppsw::to_string(orientation).c_str());
  }
  ER_CHECK(angle_diff <= kROBOT_AZIMUTH_TOL,
           "Robot azimuth (%s) too far from lane orientation (%s): %s > %s",
           rcppsw::to_string(saa()->sensing()->azimuth()).c_str(),
           rcppsw::to_string(orientation).c_str(),
           rcppsw::to_string(angle_diff).c_str(),
           rcppsw::to_string(kROBOT_AZIMUTH_TOL).c_str());
  return true;

error:
  return false;
} /* lane_alignment_verify_azimuth() */

bool builder_util_fsm::robot_trajectory_proximity(void) const {
  auto readings =
      sensing()->sensor<chal::sensors::colored_blob_camera_sensor>()->readings();
  auto robot_dpos = saa()->sensing()->dpos2D();
  auto robot_azimuth = saa()->sensing()->azimuth();

  bool prox = false;
  for (auto& r : readings) {
    /* something other than a robot */
    if (rutils::color::kBLUE != r.color) {
      continue;
    }
    auto other_dpos =
        rmath::dvec2zvec(r.vec, mc_perception->arena_resolution().v());
    rmath::range<rmath::radians> pos_x(
        rmath::radians::kZERO + kROBOT_AZIMUTH_TOL,
        rmath::radians::kZERO - kROBOT_AZIMUTH_TOL);
    rmath::range<rmath::radians> neg_x(rmath::radians::kPI + kROBOT_AZIMUTH_TOL,
                                       rmath::radians::kPI - kROBOT_AZIMUTH_TOL);
    rmath::range<rmath::radians> pos_y(
        rmath::radians::kPI_OVER_TWO + kROBOT_AZIMUTH_TOL,
        rmath::radians::kPI_OVER_TWO - kROBOT_AZIMUTH_TOL);
    rmath::range<rmath::radians> neg_y(
        rmath::radians::kTHREE_PI_OVER_TWO + kROBOT_AZIMUTH_TOL,
        rmath::radians::kTHREE_PI_OVER_TWO - kROBOT_AZIMUTH_TOL);
    if (pos_x.contains(robot_azimuth)) {
      prox |= (other_dpos.x() - robot_dpos.x()) <= 2;
    } else if (neg_x.contains(robot_azimuth)) {
      prox |= (robot_dpos.x() - other_dpos.x()) <= 2;
    } else if (pos_y.contains(robot_azimuth)) {
      prox |= (other_dpos.y() - robot_dpos.y()) <= 2;
    } else {
      prox |= (robot_dpos.y() - robot_dpos.y()) <= 2;
    }
  } /* for(&r..) */
  return prox;
} /* robot_trajectory_proximity() */

bool builder_util_fsm::robot_manhattan_proximity(void) const {
  auto readings =
      sensing()->sensor<chal::sensors::colored_blob_camera_sensor>()->readings();
  auto robot_dpos = saa()->sensing()->dpos2D();

  bool prox = false;
  for (auto& r : readings) {
    /* something other than a robot */
    if (rutils::color::kBLUE != r.color) {
      continue;
    }
    auto other_dpos =
        rmath::dvec2zvec(r.vec, mc_perception->arena_resolution().v());
    size_t dist = std::abs(static_cast<int>(robot_dpos.x() - other_dpos.x())) +
                  std::abs(static_cast<int>(robot_dpos.y() - other_dpos.y()));
    prox |= (dist <= 2);
  } /* for(&r..) */
  return prox;
} /* robot_manhattan_proximity() */

NS_END(fsm, silicon);
