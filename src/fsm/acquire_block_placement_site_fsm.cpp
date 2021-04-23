/**
 * \file acquire_block_placement_site_fsm.cpp
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
#include "silicon/fsm/acquire_block_placement_site_fsm.hpp"

#include "rcppsw/patterns/fsm/event.hpp"

#include "cosm/ds/cell3D.hpp"
#include "cosm/spatial/fsm/util_signal.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"

#include "silicon/controller/perception/builder_perception_subsystem.hpp"
#include "silicon/fsm/calculators/ingress_path.hpp"
#include "silicon/fsm/calculators/placement_intent.hpp"
#include "silicon/fsm/calculators/placement_path.hpp"
#include "silicon/fsm/construction_signal.hpp"
#include "silicon/fsm/fs_acq_checker.hpp"
#include "silicon/repr/colors.hpp"
#include "silicon/repr/construction_lane.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
acquire_block_placement_site_fsm::acquire_block_placement_site_fsm(
    const scperception::builder_perception_subsystem* perception,
    csubsystem::saa_subsystemQ3D* saa,
    rmath::rng* rng)
    : builder_util_fsm(perception, saa, rng, fsm_state::ekST_MAX_STATES),
      ER_CLIENT_INIT("silicon.fsm.acquire_block_placement_site"),
      RCPPSW_HFSM_CONSTRUCT_STATE(wait_for_robot, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(acquire_frontier_set, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(acquire_placement_loc, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      RCPPSW_HFSM_DEFINE_STATE_MAP(
          mc_state_map,
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&start),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&acquire_frontier_set,
                                             nullptr,
                                             &entry_acquire_frontier_set,
                                             nullptr),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_robot,
                                             nullptr,
                                             &entry_wait_for_robot,
                                             &exit_wait_for_robot),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&acquire_placement_loc,
                                             nullptr,
                                             nullptr,
                                             nullptr),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&finished)),
      m_alignment_calc(saa->sensing()) {}

acquire_block_placement_site_fsm::~acquire_block_placement_site_fsm(void) =
    default;

/*******************************************************************************
 * States
 ******************************************************************************/
RCPPSW_HFSM_STATE_DEFINE_ND(acquire_block_placement_site_fsm, start) {
  return rpfsm::event_signal::ekHANDLED;
}

RCPPSW_HFSM_ENTRY_DEFINE_ND(acquire_block_placement_site_fsm,
                            entry_acquire_frontier_set) {
  /* disable proximity sensor and enable camera while on the structure */
  saa()->sensing()->blobs()->enable();
  saa()->sensing()->proximity()->disable();

  /*
   * Turn on LEDs so we can be identified by other robots while on the
   * structure.
   */
  saa()->actuation()->leds()->set_color(-1, srepr::colors::builder());
}

RCPPSW_HFSM_STATE_DEFINE_ND(acquire_block_placement_site_fsm,
                            acquire_frontier_set) {
  auto alignment = m_alignment_calc(allocated_lane());
  ER_ASSERT(alignment.ingress_pos,
            "Bad alignment (position) during frontier set acqusition");
  ER_CHECKW(alignment.azimuth,
            "Bad alignment (orientation) during frontier set acquisition");

  auto acq = fs_acq_checker(sensing(), perception())(allocated_lane());
  auto prox = perception()->builder_prox()->operator()(allocated_lane(), acq);

  /*
   * A robot in front of us is too close (either on structure or when we are in
   * the nest moving towards the structure)--wait for it to move before
   * continuing.
   */
  if (scperception::builder_prox_type::ekNONE != prox) {
    ER_DEBUG("Wait@%s/%s while acquiring frontier set: robot proximity=%d",
             rcppsw::to_string(sensing()->rpos2D()).c_str(),
             rcppsw::to_string(sensing()->dpos2D()).c_str(),
             rcppsw::as_underlying(prox));
    internal_event(fsm_state::ekST_WAIT_FOR_ROBOT,
                   std::make_unique<robot_wait_data>(prox, acq));
    return rpfsm::event_signal::ekHANDLED;
  }

  /* In nest, but not on structure yet */
  if (nullptr == perception()->los()) {
    ER_TRACE("Robot=%s/%s: No robots detected, in nest",
             rcppsw::to_string(sensing()->rpos2D()).c_str(),
             rcppsw::to_string(sensing()->dpos2D()).c_str());
    saa()->steer_force2D().accum(
        saa()->steer_force2D().path_following(m_path.get()));
  } else {
    /* somewhere on structure */
    ER_TRACE("Robot=%s/%s: On structure, acq=%d",
             rcppsw::to_string(sensing()->rpos2D()).c_str(),
             rcppsw::to_string(sensing()->dpos2D()).c_str(),
             rcppsw::as_underlying(acq));

    if (srepr::fs_configuration::ekNONE == acq) {
      /* no robots in front, but not there yet */
      saa()->steer_force2D().accum(
          saa()->steer_force2D().path_following(m_path.get()));
    } else { /* arrived! */
      auto calculator = calculators::placement_path(sensing(), perception());
      m_path = std::make_unique<csteer2D::ds::path_state>(
          calculator(allocated_lane(), acq));
      internal_event(fsm_state::ekST_ACQUIRE_PLACEMENT_LOC);
    }
  }
  return rpfsm::event_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE_ND(acquire_block_placement_site_fsm,
                            acquire_placement_loc) {
  auto acq = fs_acq_checker(sensing(), perception())(allocated_lane());
  auto prox = perception()->builder_prox()->operator()(allocated_lane(), acq);

  if (scperception::builder_prox_type::ekNONE != prox) {
    ER_DEBUG("Wait@%s/%s while acquiring frontier set: robot proximity=%d",
             rcppsw::to_string(sensing()->rpos2D()).c_str(),
             rcppsw::to_string(sensing()->dpos2D()).c_str(),
             rcppsw::as_underlying(prox));
    internal_event(fsm_state::ekST_WAIT_FOR_ROBOT,
                   std::make_unique<robot_wait_data>(prox, acq));
  } else {
    if (!m_path->is_complete()) {
      auto force = saa()->steer_force2D().path_following(m_path.get());
      saa()->steer_force2D().accum(force);
    } else {
      m_path.reset();
      internal_event(fsm_state::ekST_FINISHED);
    }
  }
  return rpfsm::event_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE_ND(acquire_block_placement_site_fsm, finished) {
  if (fsm_state::ekST_FINISHED != last_state()) {
    ER_DEBUG("Executing ekST_FINISHED");
  }
  return rpfsm::event_signal::ekHANDLED;
}

RCPPSW_HFSM_ENTRY_DEFINE_ND(acquire_block_placement_site_fsm,
                            entry_wait_for_robot) {
  inta_tracker()->inta_enter();
}

RCPPSW_HFSM_EXIT_DEFINE(acquire_block_placement_site_fsm, exit_wait_for_robot) {
  inta_tracker()->inta_exit();
}

/*******************************************************************************
 * Taskable Interface
 ******************************************************************************/
void acquire_block_placement_site_fsm::task_start(cta::taskable_argument* c_arg) {
  static const std::array<uint8_t, fsm_state::ekST_MAX_STATES> kTRANSITIONS = {
    fsm_state::ekST_ACQUIRE_FRONTIER_SET, /* start */
    rpfsm::event_signal::ekFATAL, /* acquire_frontier_set */
    rpfsm::event_signal::ekFATAL, /* wait_for_robot */
    rpfsm::event_signal::ekFATAL, /* acquire placement loc */
    fsm_state::ekST_ACQUIRE_FRONTIER_SET, /* finished */
  };
  RCPPSW_HFSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, fsm_state::ekST_MAX_STATES);

  auto* const a = dynamic_cast<repr::construction_lane*>(c_arg);
  ER_ASSERT(nullptr != a, "Bad construction lane argument");
  allocated_lane(a);

  auto calculator = calculators::ingress_path(sensing(), perception());
  m_path =
      std::make_unique<csteer2D::ds::path_state>(calculator(allocated_lane()));
  external_event(kTRANSITIONS[current_state()]);
} /* task_start() */

void acquire_block_placement_site_fsm::task_execute(void) {
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
repr::placement_intent
acquire_block_placement_site_fsm::placement_intent_calc(void) const {
  ER_ASSERT(fsm_state::ekST_FINISHED == current_state(),
            "Placement cell can only be calculated once the FSM finishes");
  auto calculator = calculators::placement_intent(sensing(), perception());
  return calculator(allocated_lane());
} /* placement_intent_calc() */

NS_END(fsm, silicon);
