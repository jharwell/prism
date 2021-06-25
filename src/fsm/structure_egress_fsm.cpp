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

#include "cosm/ds/cell3D.hpp"
#include "cosm/spatial/fsm/util_signal.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

#include "silicon/controller/perception/builder_perception_subsystem.hpp"
#include "silicon/fsm/calculators/egress_lane_path.hpp"
#include "silicon/fsm/calculators/egress_path.hpp"
#include "silicon/fsm/construction_signal.hpp"
#include "silicon/repr/colors.hpp"
#include "silicon/repr/construction_lane.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
structure_egress_fsm::structure_egress_fsm(
    const scperception::builder_perception_subsystem* perception,
    csubsystem::saa_subsystemQ3D* saa,
    rmath::rng* rng)
    : builder_util_fsm(perception, saa, rng, fsm_state::ekST_MAX_STATES),
      ER_CLIENT_INIT("silicon.fsm.egress"),
      RCPPSW_HFSM_CONSTRUCT_STATE(wait_for_robot, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(acquire_egress_lane, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(structure_egress, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      RCPPSW_HFSM_DEFINE_STATE_MAP(
          mc_state_map,
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&start),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&acquire_egress_lane,
                                             nullptr,
                                             &entry_acquire_egress_lane,
                                             nullptr),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&structure_egress,
                                             nullptr,
                                             &entry_structure_egress,
                                             nullptr),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_robot,
                                             nullptr,
                                             &entry_wait_for_robot,
                                             &exit_wait_for_robot),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&finished,
                                             nullptr,
                                             &entry_finished,
                                             nullptr)),
      m_alignment_calc(saa->sensing()) {}

/*******************************************************************************
 * States
 ******************************************************************************/
RCPPSW_HFSM_STATE_DEFINE_ND(structure_egress_fsm, start) {
  return rpfsm::event_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE_ND(structure_egress_fsm, acquire_egress_lane) {
  auto prox_checker = perception()->builder_prox();
  auto prox =
      prox_checker->operator()(allocated_lane(), srepr::fs_configuration::ekNONE);

  if (scperception::builder_prox_type::ekNONE != prox) {
    ER_DEBUG("Wait@%s/%s while acquiring egress lane: robot manhattan proximity",
             rcppsw::to_string(sensing()->rpos2D()).c_str(),
             rcppsw::to_string(sensing()->dpos2D()).c_str())
    internal_event(ekST_WAIT_FOR_ROBOT, std::make_unique<robot_wait_data>(prox));
  } else {
    if (m_egress_state->is_complete()) {
      auto egress_pt = allocated_lane()->geometry().egress_pt();
      ER_DEBUG("Acquired egress lane%zu egress@%s",
               allocated_lane()->id(),
               rcppsw::to_string(egress_pt).c_str());
      auto calculator = calculators::egress_path(sensing(), perception(), rng());
      auto path = calculator(allocated_lane());
      m_egress_state = std::make_unique<csteer2D::ds::path_state>(path);
      internal_event(ekST_STRUCTURE_EGRESS);
    } else {
      auto force = saa()->steer_force2D().path_following(m_egress_state.get());
      saa()->steer_force2D().accum(force);
    }
  }
  return rpfsm::event_signal::ekHANDLED;
}

RCPPSW_HFSM_ENTRY_DEFINE_ND(structure_egress_fsm, entry_acquire_egress_lane) {
  /*
   * Turn on LEDs--they were turned off when the task was reset. I don't think
   * this is a design flaw--it is reasonable for an FSM to want to reset all
   * actuators to their initial state and not rely on carryover from whatever
   * previous task/FSM was running. Things are more modular this way.
   */
  saa()->actuation()->leds()->set_color(-1, srepr::colors::builder());
}

RCPPSW_HFSM_STATE_DEFINE_ND(structure_egress_fsm, structure_egress) {
  auto alignment = m_alignment_calc(allocated_lane());
  bool in_ct_zone = saa()->sensing()->ground()->detect("nest");
  ER_ASSERT(!(in_ct_zone && !alignment.egress),
            "Bad alignment (position) on structure egress");

  /*
   * A robot in front of us is too close--wait for it to move before
   * continuing.
   */
  if (in_ct_zone) {
    auto prox_checker = perception()->builder_prox();
    auto prox = prox_checker->operator()(allocated_lane(),
                                         srepr::fs_configuration::ekNONE);

    if (scperception::builder_prox_type::ekNONE != prox) {
      ER_DEBUG("Wait@%s/%s structure egress: robot trajectory proximity",
               rcppsw::to_string(sensing()->rpos2D()).c_str(),
               rcppsw::to_string(sensing()->dpos2D()).c_str())
      internal_event(ekST_WAIT_FOR_ROBOT,
                     std::make_unique<robot_wait_data>(prox));
    } else {
      auto force = saa()->steer_force2D().path_following(m_egress_state.get());
      saa()->steer_force2D().accum(force);
    }
  } else { /* left construction zone */
    if (m_egress_state->is_complete()) {
      internal_event(ekST_FINISHED);
    } else {
      auto force = saa()->steer_force2D().path_following(m_egress_state.get());
      saa()->steer_force2D().accum(force);

      /* back in 2D arena, so go back to obstacle avoidance */
      if (auto obs = saa()->sensing()->proximity()->avg_prox_obj()) {
        saa()->steer_force2D().accum(saa()->steer_force2D().avoidance(*obs));
      }
    }
  }
  return rpfsm::event_signal::ekHANDLED;
}

RCPPSW_HFSM_ENTRY_DEFINE_ND(structure_egress_fsm, entry_structure_egress) {
  /*
   * Enable proximity sensor, which is needed as we exit the nest/construction
   * zone and move back into the 2D arena as part of structure egress.
   */
  saa()->sensing()->proximity()->enable();

  /*
   * Enable the camera, which may have been disabled previously during egress
   * if we had to wait for a robot while in the egress lane.
   */
  saa()->sensing()->blobs()->enable();

  /*
   * Turn on LEDs--they may have been turned off when we exited the egress state
   * due to robot proximity.
   */
  saa()->actuation()->leds()->set_color(-1, srepr::colors::builder());
}

RCPPSW_HFSM_STATE_DEFINE_ND(structure_egress_fsm, finished) {
  if (ekST_FINISHED != last_state()) {
    ER_DEBUG("Executing ekST_FINISHED");
  }
  return rpfsm::event_signal::ekHANDLED;
}

RCPPSW_HFSM_ENTRY_DEFINE_ND(structure_egress_fsm, entry_finished) {
  /*
   * If we have left the structure then disable the camera, as that is
   * computationally expensive to compute readings for in large swarms, and we
   * don't need it anymore at this point.
   */
  saa()->sensing()->blobs()->disable();
}
/*******************************************************************************
 * Taskable Interface
 ******************************************************************************/
void structure_egress_fsm::task_start(cta::taskable_argument* c_arg) {
  static const std::array<uint8_t, ekST_MAX_STATES> kTRANSITIONS = {
    ekST_ACQUIRE_EGRESS_LANE, /* start */
    rpfsm::event_signal::ekFATAL, /* acquire egress lane */
    rpfsm::event_signal::ekFATAL, /* structure egress */
    rpfsm::event_signal::ekFATAL, /* wait for robot */
    ekST_ACQUIRE_EGRESS_LANE, /* finished */
  };
  RCPPSW_HFSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, ekST_MAX_STATES);

  auto* const a = dynamic_cast<repr::construction_lane*>(c_arg);
  ER_ASSERT(nullptr != a, "Bad construction lane argument");
  allocated_lane(a);

  auto path = calculators::egress_lane_path(sensing())(allocated_lane());
  m_egress_state = std::make_unique<csteer2D::ds::path_state>(path);
  external_event(kTRANSITIONS[current_state()]);
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
  builder_util_fsm::init();
  m_egress_state = nullptr;
} /* init() */

NS_END(fsm, silicon);
