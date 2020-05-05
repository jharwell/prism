/**
 * \file crw_fsm.cpp
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
#include "silicon/fsm/constructor_fsm.hpp"

#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"
#include "cosm/spatial/expstrat/base_expstrat.hpp"
#include "cosm/spatial/expstrat/crw.hpp"

#include "silicon/fsm/construction_signal.hpp"
#include "silicon/lane_alloc/allocator.hpp"
#include "silicon/fsm/construction_acq_goal.hpp"
#include "silicon/controller/perception/builder_perception_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
constructor_fsm::constructor_fsm(const slaconfig::lane_alloc_config* allocator_config,
                                 const scperception::builder_perception_subsystem* perception,
                                 crfootbot::footbot_saa_subsystem* const saa,
                                 rmath::rng* rng)
    : util_hfsm(saa, rng, ekST_MAX_STATES),
      ER_CLIENT_INIT("silicon.fsm.depth0.crw"),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(forage, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(wait_for_block_pickup, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(transport_to_nest, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(build, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      HFSM_DEFINE_STATE_MAP(mc_state_map,
                            HFSM_STATE_MAP_ENTRY_EX(&start),
                            HFSM_STATE_MAP_ENTRY_EX(&forage),
                            HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_block_pickup,
                                                        nullptr,
                                                        &entry_wait_for_signal,
                                                        nullptr),
                            HFSM_STATE_MAP_ENTRY_EX_ALL(&transport_to_nest,
                                                        nullptr,
                                                        &entry_transport_to_nest,
                                                        &exit_transport_to_nest),
                            HFSM_STATE_MAP_ENTRY_EX(&build),
                            HFSM_STATE_MAP_ENTRY_EX(&finished)),
      mc_perception(perception),
      m_allocator(allocator_config, rng),
      m_forage_fsm(saa,
                   std::make_unique<csexpstrat::crw>(saa, rng),
                    rng,
                   std::bind(&constructor_fsm::block_detected, this)),
      m_build_fsm(mc_perception, saa, rng) {}

/*******************************************************************************
 * States
 ******************************************************************************/
HFSM_STATE_DEFINE_ND(constructor_fsm, start) {
  internal_event(ekST_FORAGE);
  return fsm::construction_signal::ekHANDLED;
}

HFSM_STATE_DEFINE_ND(constructor_fsm, forage) {
  if (m_forage_fsm.task_finished()) {
    internal_event(ekST_WAIT_FOR_BLOCK_PICKUP);
  } else {
    m_forage_fsm.task_execute();
  }
  return fsm::construction_signal::ekHANDLED;
}

HFSM_STATE_DEFINE(constructor_fsm,
                  wait_for_block_pickup,
                  rpfsm::event_data* data) {
  if (fsm::construction_signal::ekFORAGING_BLOCK_PICKUP == data->signal()) {
    m_forage_fsm.task_reset();
    ER_INFO("Block pickup signal received while foraging");

    /* allocate construction lane */
    m_lane = m_allocator(sensing()->rpos3D(),
                         mc_perception->target());
    auto path = calc_transport_path(m_lane);
    auto transport_data = std::make_unique<nest_transport_data>(
        csteer2D::ds::path_state(path),
        m_lane.ingress(),
        sensing()->rpos3D());
    internal_event(ekST_TRANSPORT_TO_NEST, std::move(transport_data));
  } else if (fsm::construction_signal::ekFORAGING_BLOCK_VANISHED == data->signal()) {
    m_forage_fsm.task_reset();
    ER_INFO("Block vanished signal received while foraging");
    internal_event(ekST_FORAGE);
  }
  return fsm::construction_signal::ekHANDLED;
}


HFSM_STATE_DEFINE(constructor_fsm,
                  transport_to_nest,
                  nest_transport_data* data) {
  auto * light = sensing()->sensor<chal::sensors::light_sensor>();
  auto light_force = saa()->steer_force2D().phototaxis(light->readings());
  auto path_force = saa()->steer_force2D().path_following(&data->path);

  /*
   * The closer the robot gets to the target structure, the more important it is
   * that it is EXACTLY aligned with its chosen construction lane. So, apply a
   * weighting factor to the two steering forces in order to achieve this.
   */
  double factor = (sensing()->rpos3D() - data->ingress).length() /
                  (data->transport_start - data->ingress).length();
  saa()->steer_force2D().accum(light_force * factor + path_force * (1.0 - factor));
  if (sensing()->sensor<chal::sensors::ground_sensor>()->detect("nest")) {
    internal_event(ekST_BUILD);
  }
  return fsm::construction_signal::ekHANDLED;
}

HFSM_STATE_DEFINE_ND(constructor_fsm, build) {
  if (!m_build_fsm.task_running()) {
    m_build_fsm.task_reset();
    m_build_fsm.task_start(&m_lane);
  }
  if (m_build_fsm.task_finished()) {
    internal_event(ekST_FINISHED);
  } else {
    m_build_fsm.task_execute();
  }
  return fsm::construction_signal::ekHANDLED;
}

HFSM_STATE_DEFINE_ND(constructor_fsm, finished) {
  if (ekST_FINISHED != last_state()) {
    ER_DEBUG("Executing ekST_FINISHED");
  }
  return rpfsm::event_signal::ekHANDLED;
}

HFSM_ENTRY_DEFINE_ND(constructor_fsm, entry_transport_to_nest) {
  sensing()->sensor<chal::sensors::light_sensor>()->enable();
  actuation()->actuator<chal::actuators::led_actuator>()->set_color(-1,
                                                                    rutils::color::kGREEN);
}
HFSM_EXIT_DEFINE(constructor_fsm, exit_transport_to_nest) {
  sensing()->sensor<chal::sensors::light_sensor>()->disable();
}

/*******************************************************************************
 * Foraging Goal Metrics
 ******************************************************************************/
constructor_fsm::exp_status constructor_fsm::is_exploring_for_goal(void) const {
  return std::make_pair(m_forage_fsm.task_running(), true);
} /* is_exploring_for_goal() */

bool constructor_fsm::goal_acquired(void) const {
  return m_forage_fsm.task_finished();
} /* goal_acquired() */

rmath::vector2z constructor_fsm::acquisition_loc(void) const {
  return sensing()->dpos2D();
} /* acquisition_loc() */

rmath::vector2z constructor_fsm::current_explore_loc(void) const {
  return sensing()->dpos2D();
} /* current_explore_loc() */

rmath::vector2z constructor_fsm::current_vector_loc(void) const {
  return sensing()->dpos2D();
} /* current_vector_loc() */

rtypes::type_uuid constructor_fsm::entity_acquired_id(void) const {
  return rtypes::constants::kNoUUID;
} /* entity_acquired_id() */

csmetrics::goal_acq_metrics::goal_type constructor_fsm::acquisition_goal(void) const {
  if (ekST_FORAGE == current_state() ||
      ekST_WAIT_FOR_BLOCK_PICKUP == current_state()) {
    return fsm::to_goal_type(construction_acq_goal::ekBLOCK);
  }
  return fsm::to_goal_type(construction_acq_goal::ekNONE);
} /* block_transport_goal() */

/*******************************************************************************
 * Foraging Collision Metrics
 ******************************************************************************/
bool constructor_fsm::in_collision_avoidance(void) const {
  return (m_forage_fsm.task_running() &&
          m_forage_fsm.in_collision_avoidance());
} /* in_collision_avoidance() */

bool constructor_fsm::entered_collision_avoidance(void) const {
  return (m_forage_fsm.task_running() &&
          m_forage_fsm.entered_collision_avoidance());
} /* entered_collision_avoidance() */

bool constructor_fsm::exited_collision_avoidance(void) const {
  return (m_forage_fsm.task_running() &&
          m_forage_fsm.exited_collision_avoidance());
} /* exited_collision_avoidance() */

rtypes::timestep constructor_fsm::collision_avoidance_duration(void) const {
  if (m_forage_fsm.task_running()) {
    return m_forage_fsm.collision_avoidance_duration();
  } else {
    return rtypes::timestep(0);
  }
} /* collision_avoidance_duration() */

rmath::vector2z constructor_fsm::avoidance_loc2D(void) const {
  return saa()->sensing()->dpos2D();
} /* avoidance_loc2D() */

rmath::vector3z constructor_fsm::avoidance_loc3D(void) const {
  return saa()->sensing()->dpos3D();
} /* avoidance_loc3D() */

/*******************************************************************************
 * Block Transport Metrics
 ******************************************************************************/
construction_transport_goal constructor_fsm::block_transport_goal(void) const {
  if (ekST_TRANSPORT_TO_NEST == current_state()) {
    return construction_transport_goal::ekNEST;
  } else if (ekST_BUILD == current_state()) {
    return construction_transport_goal::ekSTRUCTURE;
  }
  return construction_transport_goal::ekNONE;
} /* block_transport_goal() */

/*******************************************************************************
 * Taskable Interface
 ******************************************************************************/
void constructor_fsm::task_execute(void) {
  inject_event(fsm::construction_signal::ekRUN, rpfsm::event_type::ekNORMAL);
} /* task_execute() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void constructor_fsm::init(void) {
  csfsm::util_hfsm::init();
  m_forage_fsm.init();
  m_build_fsm.init();
} /* init() */

bool constructor_fsm::block_detected(void) const {
  return saa()->sensing()->sensor<chal::sensors::ground_sensor>()->detect(
      "block");
} /* block_detected() */

std::vector<rmath::vector2d> constructor_fsm::calc_transport_path(
    const repr::construction_lane& lane) const {
  std::vector<rmath::vector2d> ret;
  if (rmath::radians::kZERO == lane.orientation()) {
    ret.push_back({sensing()->rpos2D().x(), lane.ingress().y()});
  } else if (rmath::radians::kPI_OVER_TWO == lane.orientation()) {
    ret.push_back({lane.ingress().x(), sensing()->rpos2D().y()});
  } else {
    ER_FATAL_SENTINEL("Bad orientation: '%s'",
                      lane.orientation().to_str().c_str());
  }
  return ret;
} /* calc_transport_path() */

NS_END(fsm, silicon);
