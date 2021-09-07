/**
 * \file fcrw_bst_fsm.cpp
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
#include "silicon/fsm/fcrw_bst_fsm.hpp"

#include "cosm/spatial/strategy/base_strategy.hpp"
#include "cosm/spatial/strategy/explore/crw.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"

#include "silicon/controller/perception/builder_perception_subsystem.hpp"
#include "silicon/fsm/calculators/lane_alignment.hpp"
#include "silicon/fsm/construction_acq_goal.hpp"
#include "silicon/fsm/construction_signal.hpp"
#include "silicon/lane_alloc/lane_allocator.hpp"
#include "silicon/repr/construction_lane.hpp"
#include "silicon/structure/structure3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, fsm);

/*******************************************************************************
 * Fcrw_Bsts/Destructors
 ******************************************************************************/
fcrw_bst_fsm::fcrw_bst_fsm(
    const slaconfig::lane_alloc_config* allocator_config,
    const scperception::builder_perception_subsystem* perception,
    csubsystem::saa_subsystemQ3D* const saa,
    rmath::rng* rng)
    : builder_util_fsm(perception, saa, rng, ekST_MAX_STATES),
      ER_CLIENT_INIT("silicon.fsm.fcrw_bst"),
      RCPPSW_HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(forage, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(wait_for_block_pickup, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(wait_for_block_place, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(structure_ingress, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(structure_build, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(structure_egress, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      RCPPSW_HFSM_DEFINE_STATE_MAP(
          mc_state_map,
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&start),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&forage),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_block_pickup,
                                             nullptr,
                                             &entry_wait_for_signal,
                                             nullptr),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&structure_ingress),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_block_place,
                                             nullptr,
                                             &entry_wait_for_signal,
                                             nullptr),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&structure_build),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&structure_egress),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&finished)),
      m_allocator(allocator_config, rng),
      m_allocated_lane(nullptr),
      m_forage_fsm(saa,
                   std::make_unique<csstrategy::explore::crw>(saa, rng),
                   rng,
                   std::bind(&fcrw_bst_fsm::block_detected, this)),
      m_block_place_fsm(perception, saa, rng),
      m_structure_ingress_fsm(perception, saa, rng),
      m_structure_egress_fsm(perception, saa, rng) {}

fcrw_bst_fsm::~fcrw_bst_fsm(void) = default;

/*******************************************************************************
 * States
 ******************************************************************************/
RCPPSW_HFSM_STATE_DEFINE_ND(fcrw_bst_fsm, start) {
  if (fsm_state::ekST_START != last_state()) {
    ER_DEBUG("Executing ekST_START");
  }
  internal_event(ekST_FORAGE);
  return fsm::construction_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE_ND(fcrw_bst_fsm, forage) {
  if (fsm_state::ekST_FORAGE != last_state()) {
    ER_DEBUG("Executing ekST_FORAGE");
  }

  if (!m_forage_fsm.task_running()) {
    m_forage_fsm.task_reset();
    m_forage_fsm.task_start(nullptr);
  }
  m_forage_fsm.task_execute();

  if (m_forage_fsm.task_finished()) {
    ER_DEBUG("Foraging finished");
    internal_event(ekST_WAIT_FOR_BLOCK_PICKUP);
  }
  return fsm::construction_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE(fcrw_bst_fsm,
                         wait_for_block_pickup,
                         rpfsm::event_data* data) {
  if (fsm::construction_signal::ekFORAGING_BLOCK_PICKUP == data->signal()) {
    ER_INFO("Block pickup signal received while foraging");
    /* allocate construction lane */
    ER_ASSERT(nullptr != perception()->nearest_ct(),
              "Cannot allocate construction lane: No known construction targets");

    m_allocated_lane =
        m_allocator(sensing()->rpos3D(), perception()->nearest_ct());
    internal_event(ekST_STRUCTURE_INGRESS);
  } else if (fsm::construction_signal::ekFORAGING_BLOCK_VANISHED == data->signal()) {
      ER_INFO("Block vanished signal received while foraging");
      internal_event(ekST_FORAGE);
  }
  return fsm::construction_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE_ND(fcrw_bst_fsm, structure_ingress) {
  if (fsm_state::ekST_STRUCTURE_INGRESS != last_state()) {
    ER_DEBUG("Executing ekST_STRUCTURE_INGRESS");
  }

  if (!m_structure_ingress_fsm.task_running()) {
    m_structure_ingress_fsm.task_reset();
    m_structure_ingress_fsm.task_start(m_allocated_lane.get());
  }
  m_structure_ingress_fsm.task_execute();

  if (m_structure_ingress_fsm.task_finished()) {
    ER_DEBUG("Structure ingress finished");
    internal_event(ekST_STRUCTURE_BUILD);
  }
  return fsm::construction_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE_ND(fcrw_bst_fsm, structure_build) {
  if (m_block_place_fsm.task_finished()) {
    /* note no reset call here! */
    internal_event(ekST_WAIT_FOR_BLOCK_PLACE);
  } else {
    if (!m_block_place_fsm.task_running()) {
      ER_DEBUG("Begin block placement FSM");
      m_block_place_fsm.task_start(m_allocated_lane.get());
    } else {
      m_block_place_fsm.task_execute();
    }
  }
  return fsm::construction_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE(fcrw_bst_fsm,
                         wait_for_block_place,
                         rpfsm::event_data* data) {
  if (fsm::construction_signal::ekCT_BLOCK_PLACE == data->signal()) {
    ER_INFO("Block placement signal received while building");
    /*
     * Can't reset this FSM before getting here because the placement info can
     * only be calculated when it is in the FINISHED state.
     */
    m_block_place_fsm.task_reset();
    internal_event(ekST_STRUCTURE_EGRESS);
  }
  return fsm::construction_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE_ND(fcrw_bst_fsm, structure_egress) {
  if (m_structure_egress_fsm.task_finished()) {
    m_structure_egress_fsm.task_reset();
    internal_event(ekST_FINISHED);
  } else {
    if (!m_structure_egress_fsm.task_running()) {
      m_structure_egress_fsm.task_start(m_allocated_lane.get());
    } else {
      m_structure_egress_fsm.task_execute();
    }
  }
  return fsm::construction_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE_ND(fcrw_bst_fsm, finished) {
  if (ekST_FINISHED != last_state()) {
    ER_DEBUG("Executing ekST_FINISHED");
  }
  /* repeat! */
  internal_event(ekST_FORAGE);
  return rpfsm::event_signal::ekHANDLED;
}

/*******************************************************************************
 * Goal Metrics
 ******************************************************************************/
fcrw_bst_fsm::exp_status fcrw_bst_fsm::is_exploring_for_goal(void) const {
  return exp_status{ m_forage_fsm.task_running(), true };
} /* is_exploring_for_goal() */

bool fcrw_bst_fsm::goal_acquired(void) const {
  return (ekST_WAIT_FOR_BLOCK_PICKUP == current_state()) ||
         (ekST_WAIT_FOR_BLOCK_PLACE == current_state());
} /* goal_acquired() */

rmath::vector3z fcrw_bst_fsm::acquisition_loc3D(void) const {
  return sensing()->dpos3D();
} /* acquisition_loc3D() */

rmath::vector3z fcrw_bst_fsm::explore_loc3D(void) const {
  return sensing()->dpos3D();
} /* explore_loc3D() */

rmath::vector3z fcrw_bst_fsm::vector_loc3D(void) const {
  return sensing()->dpos3D();
} /* vector_loc3D() */

rtypes::type_uuid fcrw_bst_fsm::entity_acquired_id(void) const {
  return rtypes::constants::kNoUUID;
} /* entity_acquired_id() */

csmetrics::goal_acq_metrics::goal_type
fcrw_bst_fsm::acquisition_goal(void) const {
  if (ekST_FORAGE == current_state() ||
      ekST_WAIT_FOR_BLOCK_PICKUP == current_state()) {
    return fsm::to_goal_type(construction_acq_goal::ekFORAGING_BLOCK);
  } else if (ekST_STRUCTURE_BUILD == current_state() ||
             ekST_WAIT_FOR_BLOCK_PLACE == current_state()) {
    return fsm::to_goal_type(construction_acq_goal::ekCT_BLOCK_PLACEMENT_SITE);
  }
  return fsm::to_goal_type(construction_acq_goal::ekNONE);
} /* block_transport_goal() */

/*******************************************************************************
 * Foraging Collision Metrics
 ******************************************************************************/
bool fcrw_bst_fsm::exp_interference(void) const {
  if (m_forage_fsm.task_running()) {
    return m_forage_fsm.exp_interference();
  } else if (m_structure_ingress_fsm.task_running()) {
    return m_structure_ingress_fsm.exp_interference();
  } else if (m_block_place_fsm.task_running()) {
    return m_block_place_fsm.exp_interference();
  } else if (m_structure_egress_fsm.task_running()) {
    return m_structure_egress_fsm.exp_interference();
  } else {
    return false;
  }
} /* exp_interference() */

bool fcrw_bst_fsm::entered_interference(void) const {
  if (m_forage_fsm.task_running()) {
    return m_forage_fsm.entered_interference();
  } else if (m_structure_ingress_fsm.task_running()) {
    return m_structure_ingress_fsm.entered_interference();
  } else if (m_block_place_fsm.task_running()) {
    return m_block_place_fsm.entered_interference();
  } else if (m_structure_egress_fsm.task_running()) {
    return m_structure_egress_fsm.entered_interference();
  } else {
    return false;
  }
} /* entered_interference() */

bool fcrw_bst_fsm::exited_interference(void) const {
  if (m_forage_fsm.task_running()) {
    return m_forage_fsm.exited_interference();
  } else if (m_structure_ingress_fsm.task_running()) {
    return m_structure_ingress_fsm.exited_interference();
  } else if (m_block_place_fsm.task_running()) {
    return m_block_place_fsm.exited_interference();
  } else if (m_structure_egress_fsm.task_running()) {
    return m_structure_egress_fsm.exited_interference();
  } else {
    return false;
  }
} /* exited_interference() */

rtypes::timestep fcrw_bst_fsm::interference_duration(void) const {
  if (m_forage_fsm.task_running()) {
    return m_forage_fsm.interference_duration();
  } else if (m_structure_ingress_fsm.task_running()) {
    return m_structure_ingress_fsm.interference_duration();
  } else if (m_block_place_fsm.task_running()) {
    return m_block_place_fsm.interference_duration();
  } else if (m_structure_egress_fsm.task_running()) {
    return m_structure_egress_fsm.interference_duration();
  } else {
    return rtypes::timestep(0);
  }
} /* interference_duration() */

rmath::vector3z fcrw_bst_fsm::interference_loc3D(void) const {
  return saa()->sensing()->dpos3D();
} /* interference_loc3D() */

/*******************************************************************************
 * Block Transport Metrics
 ******************************************************************************/
construction_transport_goal fcrw_bst_fsm::block_transport_goal(void) const {
  if (ekST_STRUCTURE_INGRESS == current_state()) {
    return construction_transport_goal::ekCONSTRUCTION_SITE;
  } else if (ekST_STRUCTURE_BUILD == current_state()) {
    return construction_transport_goal::ekCT_BLOCK_PLACEMENT_SITE;
  }
  return construction_transport_goal::ekNONE;
} /* block_transport_goal() */

/*******************************************************************************
 * Taskable Interface
 ******************************************************************************/
void fcrw_bst_fsm::task_execute(void) {
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
void fcrw_bst_fsm::init(void) {
  sfsm::builder_util_fsm::init();
  m_forage_fsm.init();
  m_block_place_fsm.init();
  m_structure_egress_fsm.init();
} /* init() */

bool fcrw_bst_fsm::block_detected(void) const {
  return saa()->sensing()->ground()->detect("block");
} /* block_detected() */

boost::optional<repr::placement_intent>
fcrw_bst_fsm::block_placement_intent(void) const {
  if (ekST_WAIT_FOR_BLOCK_PLACE == current_state()) {
    return boost::make_optional(m_block_place_fsm.placement_intent_calc());
  }
  return boost::none;
} /* block_placement_intent() */

NS_END(fsm, silicon);
