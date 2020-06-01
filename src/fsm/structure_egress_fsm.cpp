/**
 * \file structure_egress_fsm.cpp
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
#include "silicon/fsm/structure_egress_fsm.hpp"

#include "rcppsw/patterns/fsm/event.hpp"

#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"
#include "cosm/robots/footbot/footbot_sensing_subsystem.hpp"
#include "cosm/spatial/fsm/util_signal.hpp"
#include "cosm/ds/cell3D.hpp"

#include "silicon/fsm/construction_signal.hpp"
#include "silicon/controller/perception/builder_perception_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
structure_egress_fsm::structure_egress_fsm(const scperception::builder_perception_subsystem* perception,
                         crfootbot::footbot_saa_subsystem* saa,
                         rmath::rng* rng)
    : builder_util_fsm(perception, saa, rng, fsm_state::ekST_MAX_STATES),
      ER_CLIENT_INIT("silicon.fsm.egress"),
      HFSM_CONSTRUCT_STATE(wait_for_robot, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(acquire_egress_lane, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(structure_egress, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      HFSM_DEFINE_STATE_MAP(
          mc_state_map,
          HFSM_STATE_MAP_ENTRY_EX(&start),
          HFSM_STATE_MAP_ENTRY_EX_ALL(&acquire_egress_lane,
                                      nullptr,
                                      nullptr,
                                      nullptr),
          HFSM_STATE_MAP_ENTRY_EX_ALL(&structure_egress,
                                     nullptr,
                                     nullptr,
                                     &exit_structure_egress),
          HFSM_STATE_MAP_ENTRY_EX(&wait_for_robot),
          HFSM_STATE_MAP_ENTRY_EX(&finished)) {}

/*******************************************************************************
 * States
 ******************************************************************************/
HFSM_STATE_DEFINE_ND(structure_egress_fsm, start) {
  return rpfsm::event_signal::ekHANDLED;
}

HFSM_STATE_DEFINE(structure_egress_fsm,
                  acquire_egress_lane,
                  csteer2D::ds::path_state* path) {
  if (robot_manhattan_proximity()) {
    internal_event(ekST_WAIT_FOR_ROBOT,
                   std::make_unique<robot_wait_data>(robot_proximity_type::ekMANHATTAN));
  } else {
    if (path->is_complete()) {
      auto egress_path = calc_egress_path();
      event_data_hold(false);
      internal_event(ekST_STRUCTURE_EGRESS,
                     std::make_unique<csteer2D::ds::path_state>(egress_path));
    } else {
      event_data_hold(true);
      auto force = saa()->steer_force2D().path_following(path);
      saa()->steer_force2D().accum(force);
    }
  }
  return rpfsm::event_signal::ekHANDLED;
}

HFSM_STATE_DEFINE(structure_egress_fsm,
                  structure_egress,
                  csteer2D::ds::path_state* path) {
  ER_ASSERT(lane_alignment_verify_pos(lane()->egress().to_2D(),
                                      lane()->orientation()),
            "Bad alignment (position) on structure egress");
  ER_ASSERT(lane_alignment_verify_pos(lane()->egress().to_2D(),
                                      lane()->orientation()),
            "Bad alignment (orientation) on structure egress");
  auto* ct = perception()->nearest_ct();
  bool in_ct_zone = ct->xrange().contains(saa()->sensing()->rpos2D().x()) &&
                    ct->yrange().contains(saa()->sensing()->rpos2D().y()) &&
                    saa()->sensing()->sensor<chal::sensors::ground_sensor>()->detect("nest");
  /*
   * A robot in front of us is too close--wait for it to move before
   * continuing.
   */
  if (in_ct_zone && robot_trajectory_proximity()) {
    internal_event(ekST_WAIT_FOR_ROBOT,
                   std::make_unique<robot_wait_data>(robot_proximity_type::ekTRAJECTORY));
  } else { /* left construction zone */
    if (path->is_complete()) {
      internal_event(ekST_FINISHED);
    } else {
      event_data_hold(true);
      auto force = saa()->steer_force2D().path_following(path);
      saa()->steer_force2D().accum(force);

      /* back in 2D arena, so go back to obstacle avoidance */
      auto * prox = saa()->sensing()->sensor<chal::sensors::proximity_sensor>();
      if (auto obs = prox->avg_prox_obj()) {
        saa()->steer_force2D().accum(saa()->steer_force2D().avoidance(*obs));
      }
    }
  }
  return rpfsm::event_signal::ekHANDLED;
}

HFSM_EXIT_DEFINE(structure_egress_fsm, exit_structure_egress) {
  /*
   * If we have left the structure then disable the camera, as that is
   * computationally expensive to compute readings for in large swarms, and we
   * don't need it anymore at this point.
   */
  saa()->sensing()->sensor<chal::sensors::colored_blob_camera_sensor>()->disable();
}

HFSM_STATE_DEFINE_ND(structure_egress_fsm, finished) {
  if (ekST_FINISHED != last_state()) {
    ER_DEBUG("Executing ekST_FINISHED");
  }

  auto* sensor = saa()->sensing()->sensor<chal::sensors::ground_sensor>();
  ER_ASSERT(!sensor->detect("nest"),
            "In finished state but still in construction zone");
  return rpfsm::event_signal::ekHANDLED;
}

/*******************************************************************************
 * Taskable Interface
 ******************************************************************************/
void structure_egress_fsm::task_start(cta::taskable_argument* c_arg) {
  static const uint8_t kTRANSITIONS[] = {
    ekST_ACQUIRE_EGRESS_LANE, /* start */
    rpfsm::event_signal::ekFATAL, /* acquire egress lane */
    rpfsm::event_signal::ekFATAL, /* structure egress */
    rpfsm::event_signal::ekFATAL, /* wait for robot */
    ekST_ACQUIRE_EGRESS_LANE,  /* finished */
  };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, ekST_MAX_STATES);

  auto* const a = dynamic_cast<repr::construction_lane*>(c_arg);
  ER_ASSERT(nullptr != a, "Bad construction lane argument");
  lane(std::move(*a));

  auto path = calc_path_to_egress();
  external_event(kTRANSITIONS[current_state()],
                 std::make_unique<csteer2D::ds::path_state>(path));
} /* task_start() */

void structure_egress_fsm::task_execute(void) {
  if (event_data_hold()) {
    auto event = event_data_release();
    event->signal(fsm::construction_signal::ekRUN);
    event->type(rpfsm::event_type::ekNORMAL);
    inject_event(std::move(event));
  } else {
    inject_event(fsm::construction_signal::ekRUN, rpfsm::event_type::ekNORMAL);
  }
} /* task_execute() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void structure_egress_fsm::init(void) {
  actuation()->reset();
  util_hfsm::init();
} /* init() */

std::vector<rmath::vector2d> structure_egress_fsm::calc_egress_path(void) {
  auto pos = saa()->sensing()->rpos2D();
  auto* ct = perception()->nearest_ct();
  std::vector<rmath::vector2d> path = {pos};

  /*
   * We add some padding to the x/y range of the construction target, because
   * the nest extends a little beyond that range on the ingress/egress face, and
   * we (ideally) want to be out of the nest when we finish structure egress.
   */
  if (rmath::radians::kZERO == lane()->orientation()) {
    double x = rng()->uniform(ct->xrange().ub() + ct->block_unit_dim() * 2.0,
                              perception()->arena_xrange().ub());
    path.push_back({x, lane()->egress().y()});
  } else if (rmath::radians::kPI_OVER_TWO == lane()->orientation()) {
    double y = rng()->uniform(ct->yrange().ub() + ct->block_unit_dim() * 2.0,
                              perception()->arena_yrange().ub());
    path.push_back({lane()->egress().x(), y});
  }
  return path;
} /* calc_egress_path() */

std::vector<rmath::vector2d> structure_egress_fsm::calc_path_to_egress(void) const {
  auto rpos = saa()->sensing()->rpos2D();
  std::vector<rmath::vector2d> path = {rpos};

  /*
   * We only have a path to the ingress lane if we are not currently in it
   * (i.e., placed our block at the back of the ingress lane).
   */
  if (rmath::radians::kZERO == lane()->orientation() &&
      !lane_alignment_verify_pos(lane()->egress().to_2D(),
                                        lane()->orientation())) {
    path.push_back({rpos.x(), lane()->egress().y()});
  } else if (rmath::radians::kPI_OVER_TWO == lane()->orientation()&&
             !lane_alignment_verify_pos(lane()->egress().to_2D(),
                                        lane()->orientation())) {
    path.push_back({rpos.x(), lane()->egress().y()});
  }
  return path;
} /* calc_path_to_egress() */

NS_END(fsm, silicon);
