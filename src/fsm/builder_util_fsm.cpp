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
#include "rcppsw/utils/maskable_enum.hpp"

#include "cosm/subsystem/saa_subsystemQ3D.hpp"

#include "silicon/controller/perception/builder_perception_subsystem.hpp"
#include "silicon/repr/construction_lane.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
builder_util_fsm::builder_util_fsm(
    const scperception::builder_perception_subsystem* perception,
    csubsystem::saa_subsystemQ3D* saa,
    rmath::rng* rng,
    uint8_t max_states)
    : util_hfsm(saa, rng, max_states),
      ER_CLIENT_INIT("silicon.fsm.builder_util"),
      RCPPSW_HFSM_CONSTRUCT_STATE(wait_for_robot, hfsm::top_state()),
      mc_perception(perception) {}

builder_util_fsm::~builder_util_fsm(void) = default;

/*******************************************************************************
 * States
 ******************************************************************************/
RCPPSW_HFSM_STATE_DEFINE(builder_util_fsm,
                         wait_for_robot,
                         robot_wait_data* data) {
  saa()->actuation()->governed_diff_drive()->reset();

  event_data_hold(false);
  auto prox_checker = mc_perception->builder_prox();
  auto prox = prox_checker->operator()(mc_alloc_lane, data->fs);
  if (!(prox & data->prox_type)) {
    ER_DEBUG("Resume previous state: no proxmity of type=%d",
             rcppsw::as_underlying(data->prox_type));
    internal_event(previous_state());
  } else {
    event_data_hold(true);
  }
  return rpfsm::event_signal::ekHANDLED;
}

RCPPSW_HFSM_ENTRY_DEFINE_ND(builder_util_fsm, entry_wait_for_robot) {
  inta_tracker()->inta_enter();
}

RCPPSW_HFSM_EXIT_DEFINE(builder_util_fsm, exit_wait_for_robot) {
  inta_tracker()->inta_exit();
  bool in_ct_zone = saa()->sensing()->ground()->detect("nest");

  /*
   * If the interference episode we have just experienced occured while we were
   * on the structure, then add its duration to the running total for the
   * currently allocated lane.
   */
  if (in_ct_zone) {
    auto ct_interference = inta_tracker()->interference_duration();
    allocated_lane()->history()->interference_mark(allocated_lane()->id(),
                                                   ct_interference);
  }
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void builder_util_fsm::init(void) {
  actuation()->reset();
  util_hfsm::init();
} /* init() */

NS_END(fsm, silicon);
