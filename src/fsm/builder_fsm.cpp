/**
 * \file builder_fsm.cpp
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
#include "silicon/fsm/builder_fsm.hpp"

#include "cosm/repr/base_block2D.hpp"
#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"
#include "cosm/robots/footbot/footbot_sensing_subsystem.hpp"
#include "cosm/fsm/util_signal.hpp"
#include "cosm/ds/cell3D.hpp"

#include "silicon/controller/builder_perception_subsystem.hpp"
#include "silicon/fsm/building_signal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, fsm);

/*******************************************************************************
 * Class Constants
 ******************************************************************************/
const rmath::radians builder_fsm::kROBOT_AZIMUTH_TOL = rmath::radians(0.10);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
builder_fsm::builder_fsm(const controller::builder_perception_subsystem* perception,
                         crfootbot::footbot_saa_subsystem* saa,
                         rmath::rng* rng)
    : util_hfsm(saa, rng, fsm_state::ekST_MAX_STATES),
      ER_CLIENT_INIT("silicon.fsm.builder"),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(acquire_frontier_set, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(wait_for_robot, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(acquire_placement_loc, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(wait_for_block_place, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(acquire_egress_lane, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(structure_egress, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(post_structure_egress, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      HFSM_DEFINE_STATE_MAP(
          mc_state_map,
          HFSM_STATE_MAP_ENTRY_EX(&start),
          HFSM_STATE_MAP_ENTRY_EX_ALL(&acquire_frontier_set,
                                     nullptr,
                                     &entry_acquire_frontier_set,
                                     nullptr),
          HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_robot,
                                     nullptr,
                                     nullptr,
                                     nullptr),
          HFSM_STATE_MAP_ENTRY_EX_ALL(&acquire_placement_loc,
                                     nullptr,
                                     nullptr,
                                     nullptr),
          HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_block_place,
                                      nullptr,
                                      &entry_wait_for_signal,
                                      nullptr),
          HFSM_STATE_MAP_ENTRY_EX_ALL(&acquire_egress_lane,
                                      nullptr,
                                      nullptr,
                                      nullptr),
          HFSM_STATE_MAP_ENTRY_EX_ALL(&structure_egress,
                                     nullptr,
                                     nullptr,
                                     &exit_structure_egress),
          HFSM_STATE_MAP_ENTRY_EX_ALL(&post_structure_egress,
                                      nullptr,
                                      nullptr,
                                      nullptr),
          HFSM_STATE_MAP_ENTRY_EX(&finished)),
      mc_perception(perception) {}

/*******************************************************************************
 * States
 ******************************************************************************/
HFSM_STATE_DEFINE(builder_fsm, start, lane_acq_argument* data) {
  m_lane_data = *data;
  ER_ASSERT(lane_alignment_verify_pos(m_lane_data.ingress_point(),
                                      -m_lane_data.orientation()),
            "Bad alignment (position) on FSM start");
  ER_ASSERT(lane_alignment_verify_pos(m_lane_data.ingress_point(),
                                      -m_lane_data.orientation()),
            "Bad alignment (orientation) on FSM start");

  auto path = calc_frontier_set_path();
  internal_event(ekST_ACQUIRE_FRONTIER_SET,
                   std::make_unique<csteer2D::ds::path_state>(path));

  return cfsm::util_signal::ekHANDLED;
}

HFSM_ENTRY_DEFINE_ND(builder_fsm, entry_acquire_frontier_set) {
  saa()->sensing()->sensor<chal::sensors::colored_blob_camera_sensor>()->enable();
  saa()->actuation()->actuator<chal::actuators::led_actuator>()->set_color(-1,
                                                                           rutils::color::kBLUE);
}

HFSM_STATE_DEFINE(builder_fsm,
                 acquire_frontier_set,
                 csteer2D::ds::path_state* state) {
  ER_ASSERT(lane_alignment_verify_pos(m_lane_data.ingress_point(),
                                      -m_lane_data.orientation()),
            "Bad alignment (position) during frontier set acqusition");
  ER_ASSERT(lane_alignment_verify_azimuth(-m_lane_data.orientation()),
            "Bad alignment (orientation) during frontier set acquisition");

  auto acq = frontier_set_acquisition();

  /*
   * A robot in front of us is too close--wait for it to move before
   * continuing.
   */
  if (robot_trajectory_proximity()) {
    internal_event(ekST_WAIT_FOR_ROBOT,
                   std::make_unique<robot_wait_data>(robot_proximity_type::ekTRAJECTORY));
  } else if (stygmergic_configuration::ekNONE == acq) {
    /* no robots in front, but not there yet */
    saa()->steer_force2D().accum(saa()->steer_force2D().path_following(state));
    saa()->steer_force2D_apply();
  } else { /* arrived! */
    auto path = calc_placement_path(acq);
    internal_event(ekST_ACQUIRE_PLACEMENT_LOC,
                   std::make_unique<csteer2D::ds::path_state>(path));
  }
  return cfsm::util_signal::ekHANDLED;
}

HFSM_STATE_DEFINE(builder_fsm,
                  acquire_placement_loc,
                  csteer2D::ds::path_state* path) {
  /*
   * We don't check for trajectory proximity to other robots while acquiring the
   * placement location, because when we get to this part, we are guaranteed to
   * be the only robot in this state in THIS construction lane. We DO, however,
   * need to check for manhattan proximity in order to avoid potential
   * deadlocks.
   */
  if (robot_manhattan_proximity()) {
    internal_event(ekST_WAIT_FOR_ROBOT,
                   std::make_unique<robot_wait_data>(robot_proximity_type::ekMANHATTAN));
  } else {
    auto force = saa()->steer_force2D().path_following(path);
    if (force.is_pd()) {
      saa()->steer_force2D().accum(force);
      saa()->steer_force2D_apply();
    } else {
      internal_event(ekST_WAIT_FOR_BLOCK_PLACE);
    }
  }
  return cfsm::util_signal::ekHANDLED;
}

HFSM_STATE_DEFINE(builder_fsm,
                  wait_for_block_place,
                  const rpfsm::event_data* data) {
  if (building_signal::ekBLOCK_PLACE == data->signal()) {
    ER_INFO("Block placement signal received");
    auto path = calc_path_to_egress();
    internal_event(ekST_ACQUIRE_EGRESS_LANE,
                   std::make_unique<csteer2D::ds::path_state>(path));
  }
  return cfsm::util_signal::ekHANDLED;
}

HFSM_STATE_DEFINE(builder_fsm,
                  acquire_egress_lane,
                  csteer2D::ds::path_state* path) {
  if (robot_manhattan_proximity()) {
    internal_event(ekST_WAIT_FOR_ROBOT,
                   std::make_unique<robot_wait_data>(robot_proximity_type::ekMANHATTAN));
  } else {
    auto force = saa()->steer_force2D().path_following(path);
    if (force.is_pd()) {
      saa()->steer_force2D().accum(force);
      saa()->steer_force2D_apply();
    } else {
      auto egress_path = calc_egress_path();
      internal_event(ekST_STRUCTURE_EGRESS,
                     std::make_unique<csteer2D::ds::path_state>(egress_path));
    }
  }
  return cfsm::util_signal::ekHANDLED;
}

HFSM_STATE_DEFINE(builder_fsm,
                  structure_egress,
                  csteer2D::ds::path_state* path) {
  ER_ASSERT(lane_alignment_verify_pos(m_lane_data.ingress_point(),
                                      m_lane_data.orientation()),
            "Bad alignment (position) on structure egress");
  ER_ASSERT(lane_alignment_verify_pos(m_lane_data.ingress_point(),
                                      m_lane_data.orientation()),
            "Bad alignment (orientation) on structure egress");

  /*
   * A robot in front of us is too close--wait for it to move before
   * continuing.
   */
  if (robot_trajectory_proximity()) {
    internal_event(ekST_WAIT_FOR_ROBOT,
                   std::make_unique<robot_wait_data>(robot_proximity_type::ekTRAJECTORY));
  } else if (!mc_perception->structure_xrange().contains(saa()->sensing()->rpos2D().x()) &&
             !mc_perception->structure_yrange().contains(saa()->sensing()->rpos2D().y()) &&
             !saa()->sensing()->sensor<chal::sensors::ground_sensor>()->detect("nest")) {
    internal_event(ekST_POST_STRUCTURE_EGRESS,
                   std::make_unique<csteer2D::ds::path_state>(*path));
  } else { /* still on structure */
    auto force = saa()->steer_force2D().path_following(path);
    saa()->steer_force2D().accum(force);
    saa()->steer_force2D_apply();
  }
  return cfsm::util_signal::ekHANDLED;
}

HFSM_EXIT_DEFINE(builder_fsm, exit_structure_egress) {
  /*
   * If we have left the structure then disable the camera, as that is
   * computationally expensive to compute readings for in large swarms, and we
   * don't need it anymore at this point.
   */
  saa()->sensing()->sensor<chal::sensors::colored_blob_camera_sensor>()->disable();
  saa()->actuation()->actuator<chal::actuators::led_actuator>()->set_color(-1, rutils::color::kBLACK);
}

HFSM_STATE_DEFINE(builder_fsm,
                  post_structure_egress,
                  csteer2D::ds::path_state* path) {
  auto force = saa()->steer_force2D().path_following(path);
  if (force.is_pd()) {
    saa()->steer_force2D().accum(force);

    /* back in 2D arena, so go back to obstacle avoidance */
    auto * prox = saa()->sensing()->sensor<chal::sensors::proximity_sensor>();
    if (auto obs = prox->avg_prox_obj()) {
      saa()->steer_force2D().accum(saa()->steer_force2D().avoidance(*obs));
    }
    saa()->steer_force2D_apply();
  } else {
    internal_event(ekST_FINISHED);
  }
  return cfsm::util_signal::ekHANDLED;
}

HFSM_STATE_DEFINE_ND(builder_fsm, finished) {
  if (ekST_FINISHED != last_state()) {
    ER_DEBUG("Executing ekST_FINISHED");
  }

  auto* sensor = saa()->sensing()->sensor<chal::sensors::ground_sensor>();
  ER_ASSERT(!sensor->detect("nest"),
            "In finished state but still in construction zone");
  return cfsm::util_signal::ekHANDLED;
}

HFSM_STATE_DEFINE(builder_fsm, wait_for_robot, const robot_wait_data* data) {
  saa()->actuation()->actuator<ckin2D::governed_diff_drive>()->reset();

  if (robot_proximity_type::ekTRAJECTORY == data->prox_type &&
      !robot_trajectory_proximity()) {
    internal_event(previous_state());
  } else if (robot_proximity_type::ekMANHATTAN == data->prox_type &&
             !robot_manhattan_proximity()) {
    internal_event(previous_state());
  }
  return cfsm::util_signal::ekHANDLED;
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void builder_fsm::task_start(const cta::taskable_argument* c_arg) {
  static const uint8_t kTRANSITIONS[] = {
    cfsm::util_signal::ekFATAL, /* start */
    cfsm::util_signal::ekFATAL, /* acquire_frontier_set */
    cfsm::util_signal::ekFATAL, /* wait_for_robot */
    cfsm::util_signal::ekFATAL, /* acquire placement loc */
    cfsm::util_signal::ekFATAL, /* wait for block place */
    cfsm::util_signal::ekFATAL, /* acquire egress lane */
    cfsm::util_signal::ekFATAL, /* structure egress */
    cfsm::util_signal::ekFATAL, /* post structure egress */
    ekST_ACQUIRE_FRONTIER_SET,  /* finished */
  };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, ekST_MAX_STATES);

  auto* const a = dynamic_cast<const lane_acq_argument*>(c_arg);
  ER_ASSERT(nullptr != a, "Bad lane acquisition data argument");
  external_event(kTRANSITIONS[current_state()],
                 std::make_unique<lane_acq_argument>(a->ingress_point(),
                                                     a->egress_point(),
                                                     a->orientation()));
} /* task_start() */

void builder_fsm::task_execute(void) {
  inject_event(cfsm::util_signal::ekRUN, rpfsm::event_type::ekNORMAL);
} /* task_execute() */

void builder_fsm::init(void) {
  actuation()->reset();
  util_hfsm::init();
} /* init() */

bool builder_fsm::lane_alignment_verify_pos(
    const rmath::vector2d& lane_point,
    const rmath::radians& orientation) const {
  auto dist_diff = saa()->sensing()->rpos2D() - lane_point;
  if (rmath::radians::kZERO == orientation) {
    ER_CHECK(dist_diff.x() <= kLANE_VECTOR_DIST_TOL,
             "Robot too far from ingress point %s for lane orientated@'%s': %f > %f",
             lane_point.to_str().c_str(),
             orientation.to_str().c_str(),
             dist_diff.x(),
             kLANE_VECTOR_DIST_TOL);
  } else if (rmath::radians::kPI_OVER_TWO == orientation) {
    ER_CHECK(dist_diff.y() <= kLANE_VECTOR_DIST_TOL,
             "Robot too far from ingress point %s for lane orientated@'%s': %f > %f",
             lane_point.to_str().c_str(),
             orientation.to_str().c_str(),
             dist_diff.y(),
             kLANE_VECTOR_DIST_TOL);
  } else {
    ER_FATAL_SENTINEL("Bad lane orientation '%s'", orientation.to_str().c_str());
  }
  return true;

error:
  return false;
} /* lane_alignment_verify_pos() */

bool builder_fsm::lane_alignment_verify_azimuth(
    const rmath::radians& orientation) const {
  auto angle_diff = orientation - saa()->sensing()->azimuth();
  ER_CHECK(angle_diff <= kROBOT_AZIMUTH_TOL,
           "Robot orientation too far from lane orientated@'%s': '%s' > '%s",
           kROBOT_AZIMUTH_TOL.to_str().c_str(),
           angle_diff.to_str().c_str(),
           kROBOT_AZIMUTH_TOL.to_str().c_str());
  return true;

error:
  return false;
} /* lane_alignment_verify_azimuth() */

stygmergic_configuration builder_fsm::frontier_set_acquisition(void) const {
  auto pos = saa()->sensing()->dpos3D();
  rmath::vector3z ingress_abs;
  rmath::vector3z egress_abs;

  if (rmath::radians::kZERO == m_lane_data.orientation()) {
    ingress_abs = pos - rmath::vector3z::X * 2;
    egress_abs = pos - rmath::vector3z::X * 2 + rmath::vector3z::Y;
  } else if (rmath::radians::kPI_OVER_TWO == m_lane_data.orientation()) {
    ingress_abs = pos - rmath::vector3z::Y * 2;
    egress_abs = pos - rmath::vector3z::Y * 2 - rmath::vector3z::X;
  } else {
    ER_FATAL_SENTINEL("Bad lane orientation '%s'",
                      m_lane_data.orientation().to_str().c_str());
  }
  auto origin = mc_perception->los()->abs_ll();
  bool ingress_has_block = mc_perception->los()->contains_loc(ingress_abs - origin) &&
                           mc_perception->los()->access((ingress_abs - origin)).state_has_block();
  bool egress_has_block = mc_perception->los()->contains_loc(egress_abs - origin) &&
                          mc_perception->los()->access((egress_abs - origin)).state_has_block();

  if (ingress_has_block && egress_has_block) {
    return stygmergic_configuration::ekLANE_FILLED;
  } else if (ingress_has_block && !egress_has_block) {
    return stygmergic_configuration::ekLANE_GAP_EGRESS;
  } else if (!ingress_has_block && egress_has_block) {
    return stygmergic_configuration::ekLANE_GAP_INGRESS;
  } else {
    return stygmergic_configuration::ekNONE;
  }
} /* frontier_set_acquisition() */

std::vector<rmath::vector2d> builder_fsm::calc_path_to_egress(void) const {
  std::vector<rmath::vector2d> path;
  auto pos = saa()->sensing()->dpos2D();
  if (rmath::radians::kZERO == m_lane_data.orientation() &&
      !lane_alignment_verify_pos(m_lane_data.egress_point(),
                                 m_lane_data.orientation())) {
    path.push_back(rmath::zvec2dvec({pos.x(), pos.y() + 1},
                                    mc_perception->grid_resolution().v()));
  } else if (rmath::radians::kPI_OVER_TWO == m_lane_data.orientation() &&
             !lane_alignment_verify_pos(m_lane_data.egress_point(),
                                        m_lane_data.orientation())) {
    path.push_back(rmath::zvec2dvec({pos.x() - 1, pos.y()},
                                    mc_perception->grid_resolution().v()));
  }
  return path;
} /* calc_path_to_egress() */

std::vector<rmath::vector2d> builder_fsm::calc_egress_path(void) {
  std::vector<rmath::vector2d> path;
  auto pos = saa()->sensing()->rpos2D();

  if (rmath::radians::kZERO == m_lane_data.orientation()) {
    double x = rng()->uniform(mc_perception->structure_xrange().ub(),
                              mc_perception->arena_xrange().ub());
    path.push_back({x, pos.y()});
  } else if (rmath::radians::kPI_OVER_TWO == m_lane_data.orientation()) {
    double y = rng()->uniform(mc_perception->structure_yrange().ub(),
                              mc_perception->arena_yrange().ub());
    path.push_back({pos.x(), y});
  }
  return path;
} /* calc_egress_path() */

std::vector<rmath::vector2d> builder_fsm::calc_placement_path(
    stygmergic_configuration acq) const {
  std::vector<rmath::vector2d> path;
  auto pos = saa()->sensing()->dpos2D();
  if (rmath::radians::kZERO == m_lane_data.orientation()) {
    if (stygmergic_configuration::ekLANE_GAP_INGRESS == acq) {
      path.push_back(rmath::zvec2dvec({pos.x() - 1, pos.y()},
                                      mc_perception->grid_resolution().v()));
    } else if (stygmergic_configuration::ekLANE_GAP_EGRESS == acq) {
      path.push_back(rmath::zvec2dvec({pos.x() - 1, pos.y()},
                                      mc_perception->grid_resolution().v()));
      path.push_back(rmath::zvec2dvec({pos.x() - 1, pos.y() + 1},
                                      mc_perception->grid_resolution().v()));
    }
  } else if (rmath::radians::kPI_OVER_TWO == m_lane_data.orientation()) {
    if (stygmergic_configuration::ekLANE_GAP_INGRESS == acq) {
      path.push_back(rmath::zvec2dvec({pos.x(), pos.y() - 1},
                                      mc_perception->grid_resolution().v()));
    } else if (stygmergic_configuration::ekLANE_GAP_EGRESS == acq) {
      path.push_back(rmath::zvec2dvec({pos.x(), pos.y() - 1},
                                      mc_perception->grid_resolution().v()));
      path.push_back(rmath::zvec2dvec({pos.x() - 1, pos.y()},
                                      mc_perception->grid_resolution().v()));
    }
  }
  return path;
} /* calc_placement_path() */

std::vector<rmath::vector2d> builder_fsm::calc_frontier_set_path(void) const {
  std::vector<rmath::vector2d> path;

  /*
   * Simple path to go from the robots current position to the back of the
   * structure (end will never be reached). This is done so a reference to the
   * structure being created is not needed and the robot can be more
   * stygmergic.
   */
  if (rmath::radians::kZERO == m_lane_data.orientation()) {
    path.push_back({0.0, m_lane_data.ingress_point().y()});
  } else if (rmath::radians::kPI_OVER_TWO == m_lane_data.orientation()) {
    /*
     * Simple path to go from the robots current position to the back of the
     * structure (end will never be reached). This is done so a reference to the
     * structure being created is not needed and the robot can be more
     * stygmergic.
     */
    path.push_back({m_lane_data.ingress_point().x(), 0.0});
  }
  return path;
} /* calc_frontier_set_path() */

bool builder_fsm::robot_trajectory_proximity(void) const {
  auto readings =
      sensing()->sensor<chal::sensors::colored_blob_camera_sensor>()->readings();
  auto robot_dpos = saa()->sensing()->dpos2D();
  auto robot_azimuth = saa()->sensing()->azimuth();

  bool prox = false;
  for (auto &r : readings) {
    /* something other than a robot */
    if (rutils::color::kBLUE != r.color) {
      continue;
    }
    auto other_dpos = rmath::dvec2zvec(r.vec,
                                       mc_perception->grid_resolution().v());
    rmath::range<rmath::radians> pos_x(rmath::radians::kZERO + kROBOT_AZIMUTH_TOL,
                                       rmath::radians::kZERO - kROBOT_AZIMUTH_TOL);
    rmath::range<rmath::radians> neg_x(rmath::radians::kPI + kROBOT_AZIMUTH_TOL,
                                       rmath::radians::kPI - kROBOT_AZIMUTH_TOL);
    rmath::range<rmath::radians> pos_y(rmath::radians::kPI_OVER_TWO +
                                       kROBOT_AZIMUTH_TOL,
                                       rmath::radians::kPI_OVER_TWO -
                                       kROBOT_AZIMUTH_TOL);
    rmath::range<rmath::radians> neg_y(rmath::radians::kTHREE_PI_OVER_TWO +
                                       kROBOT_AZIMUTH_TOL,
                                       rmath::radians::kTHREE_PI_OVER_TWO -
                                       kROBOT_AZIMUTH_TOL);
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

bool builder_fsm::robot_manhattan_proximity(void) const {
  auto readings =
      sensing()->sensor<chal::sensors::colored_blob_camera_sensor>()->readings();
  auto robot_dpos = saa()->sensing()->dpos2D();

  bool prox = false;
  for (auto &r : readings) {
    /* something other than a robot */
    if (rutils::color::kBLUE != r.color) {
      continue;
    }
    auto other_dpos = rmath::dvec2zvec(r.vec,
                                       mc_perception->grid_resolution().v());
    size_t dist = std::abs(static_cast<int>(robot_dpos.x() - other_dpos.x())) +
                  std::abs(static_cast<int>(robot_dpos.y() - other_dpos.y()));
    prox |= (dist <= 2);
  } /* for(&r..) */
  return prox;
} /* robot_manhattan_proximity() */

NS_END(fsm, silicon);
