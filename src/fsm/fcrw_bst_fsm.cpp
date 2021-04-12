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
#include "silicon/fsm/calculators/ct_approach.hpp"
#include "silicon/fsm/calculators/ingress_lane_path.hpp"
#include "silicon/fsm/calculators/lane_alignment.hpp"
#include "silicon/fsm/construction_acq_goal.hpp"
#include "silicon/fsm/construction_signal.hpp"
#include "silicon/lane_alloc/allocator.hpp"
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
    : util_hfsm(saa, rng, ekST_MAX_STATES),
      ER_CLIENT_INIT("silicon.fsm.fcrw_bst"),
      RCPPSW_HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(forage, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(wait_for_block_pickup, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(wait_for_block_place, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(ct_approach, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(ct_entry, hfsm::top_state()),
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
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&ct_approach,
                                             nullptr,
                                             &entry_ct_approach,
                                             nullptr),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&ct_entry,
                                             nullptr,
                                             nullptr,
                                             &exit_ct_entry),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_block_place,
                                             nullptr,
                                             &entry_wait_for_signal,
                                             nullptr),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&structure_build),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&structure_egress),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&finished)),
      mc_perception(perception),
      m_allocator(allocator_config, rng),
      m_allocated_lane(nullptr),
      m_forage_fsm(saa,
                   std::make_unique<csstrategy::explore::crw>(saa, rng),
                   rng,
                   std::bind(&fcrw_bst_fsm::block_detected, this)),
      m_block_place_fsm(mc_perception, saa, rng),
      m_structure_egress_fsm(mc_perception, saa, rng) {}

fcrw_bst_fsm::~fcrw_bst_fsm(void) = default;

/*******************************************************************************
 * States
 ******************************************************************************/
RCPPSW_HFSM_STATE_DEFINE_ND(fcrw_bst_fsm, start) {
  if (fsm_states::ekST_START != last_state()) {
    ER_DEBUG("Executing ekST_START");
  }
  internal_event(ekST_FORAGE);
  return fsm::construction_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE_ND(fcrw_bst_fsm, forage) {
  if (fsm_states::ekST_FORAGE != last_state()) {
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
    ER_ASSERT(nullptr != mc_perception->nearest_ct(),
              "Cannot allocate construction lane: No known construction targets");

    /* allocate construction lane */
    m_allocated_lane =
        m_allocator(sensing()->rpos3D(), mc_perception->nearest_ct());

    auto calculator = calculators::ct_approach(
        sensing(), calculators::lane_alignment::kTRAJECTORY_ORTHOGONAL_TOL);
    auto approach = calculator(m_allocated_lane.get());

    if (!(approach.x_ok && approach.y_ok)) {
      ER_INFO("Construction target approach required");
      internal_event(ekST_CT_APPROACH);
    } else {
      auto path =
          calculators::ingress_lane_path(sensing())(m_allocated_lane.get());
      ER_INFO("Calculated transport path to lane%zu ingress with %zu waypoints",
              m_allocated_lane->id(),
              path.size());
      auto entry_data = std::make_unique<ct_ingress_data>(
          csteer2D::ds::path_state(path), m_allocated_lane->ingress());

      internal_event(ekST_CT_ENTRY, std::move(entry_data));
    }

  } else if (fsm::construction_signal::ekFORAGING_BLOCK_VANISHED ==
             data->signal()) {
    ER_INFO("Block vanished signal received while foraging");
    internal_event(ekST_FORAGE);
  }
  return fsm::construction_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE_ND(fcrw_bst_fsm, ct_approach) {
  auto calculator = calculators::ct_approach(
      sensing(), calculators::lane_alignment::kTRAJECTORY_ORTHOGONAL_TOL);
  auto approach = calculator(m_allocated_lane.get());

  if (fsm_states::ekST_CT_APPROACH != last_state()) {
    ER_DEBUG("Beginning construction target approach");

    /*
     * If the angle is >= 180 degrees, we are BELOW the X-axis for the
     * structure, and counter-clockwise to the ingress site will be faster. Vice
     * versa for < 180 degrees. See SILICON#41.
     */
    m_ct_approach_polar_sign = (approach.ingress_angle >= rmath::radians::kPI) ? -1 : 1;
  }

  if (approach.x_ok && approach.y_ok) {
    auto path = calculators::ingress_lane_path(sensing())(m_allocated_lane.get());
    ER_INFO("Calculated transport path to lane%zu ingress with %zu waypoints",
            m_allocated_lane->id(),
            path.size());
    auto data = std::make_unique<ct_ingress_data>(csteer2D::ds::path_state(path),
                                                  m_allocated_lane->ingress());

    internal_event(ekST_CT_ENTRY, std::move(data));
    event_data_hold(true);
    return fsm::construction_signal::ekHANDLED;
  }

  auto* prox = saa()->sensing()->sensor<chal::sensors::proximity_sensor>();
  if (auto obs = prox->avg_prox_obj()) {
    saa()->steer_force2D().accum(saa()->steer_force2D().avoidance(*obs));
  }

  auto* light = sensing()->sensor<chal::sensors::light_sensor>();
  auto light_force = saa()->steer_force2D().phototaxis(light->readings());

  auto polar_force =
      saa()->steer_force2D().polar(m_allocated_lane->ingress().to_2D()) *
      m_ct_approach_polar_sign;
  /*
   * We need the sign of the orthogonal distance to the ingress lane so that we
   * calculate polar force of the appropriate sign (i.e., not going all the way
   * around in a circle to reach the ingress clockwise when a short
   * counter-clockwise path is MUCH shorter).
   */
  auto* ct = mc_perception->nearest_ct();
  ER_ASSERT(nullptr != ct,
            "Cannot compute approach forces without construction target");
  /*
   * If the robot is on one of the wrong sides of the structure, we need it to
   * go around the structure in a circular path via polar force, but making
   * sure to stay out of the nest as much as possible, so we weight the polar
   * force pushing the robot away from the nest with the light force
   * attracting it to the nest in proportion to how close we get to the target
   * center.
   */
  double dist_to_center =
      (sensing()->rpos3D() - m_allocated_lane->ingress()).length();
  double light_factor = std::max(
      0.0, (dist_to_center - ct_bc_radius() * 2.0) / kCT_TRANSPORT_BC_DIST_MIN);
  saa()->steer_force2D().accum(light_force * light_factor +
                               polar_force * (1.0 - light_factor));
  saa()->steer_force2D().accum(saa()->steer_force2D().wander(rng()));
  return fsm::construction_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE(fcrw_bst_fsm, ct_entry, ct_ingress_data* data) {
  if (fsm_states::ekST_CT_ENTRY != last_state()) {
    ER_DEBUG("Beginning construction target entry");
  }

  ER_TRACE("Distance to lane%zu ingress@%s: x=%f,y=%f [%f]",
           m_allocated_lane->id(),
           rcppsw::to_string(data->ingress).c_str(),
           (sensing()->rpos3D() - data->ingress).x(),
           (sensing()->rpos3D() - data->ingress).y(),
           (sensing()->rpos3D() - data->ingress).length());

  ER_TRACE(
      "Lane%zu alignment: %s",
      m_allocated_lane->id(),
      rcppsw::to_string(sensing()->azimuth() - m_allocated_lane->orientation())
          .c_str());

  auto path_force = saa()->steer_force2D().path_following(&data->path);
  saa()->steer_force2D().accum(path_force);

  if (data->path.is_complete()) {
    ER_DEBUG("Reached lane%zu ingress@%s",
             m_allocated_lane->id(),
             rcppsw::to_string(m_allocated_lane->ingress()).c_str());
    event_data_hold(false);
    internal_event(ekST_STRUCTURE_BUILD);
  } else {
    event_data_hold(true);
  }
  return fsm::construction_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE_ND(fcrw_bst_fsm, structure_build) {
  if (m_block_place_fsm.task_finished()) {
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

RCPPSW_HFSM_ENTRY_DEFINE_ND(fcrw_bst_fsm, entry_ct_approach) {
  /* turn on light sensor to move towards the nest */
  sensing()->sensor<chal::sensors::light_sensor>()->enable();
  actuation()->actuator<chal::actuators::led_actuator>()->set_color(
      -1, rutils::color::kGREEN);
}

RCPPSW_HFSM_EXIT_DEFINE(fcrw_bst_fsm, exit_ct_entry) {
  /* Once we are on the target, we don't need the light sensor anymore. */
  sensing()->sensor<chal::sensors::light_sensor>()->disable();
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
  return (m_forage_fsm.task_running() && m_forage_fsm.exp_interference());
} /* exp_interference() */

bool fcrw_bst_fsm::entered_interference(void) const {
  return (m_forage_fsm.task_running() && m_forage_fsm.entered_interference());
} /* entered_interference() */

bool fcrw_bst_fsm::exited_interference(void) const {
  return (m_forage_fsm.task_running() && m_forage_fsm.exited_interference());
} /* exited_interference() */

rtypes::timestep fcrw_bst_fsm::interference_duration(void) const {
  if (m_forage_fsm.task_running()) {
    return m_forage_fsm.interference_duration();
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
  if (ekST_CT_APPROACH == current_state() || ekST_CT_ENTRY == current_state()) {
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
  csfsm::util_hfsm::init();
  m_forage_fsm.init();
  m_block_place_fsm.init();
  m_structure_egress_fsm.init();
} /* init() */

bool fcrw_bst_fsm::block_detected(void) const {
  return saa()->sensing()->sensor<chal::sensors::ground_sensor>()->detect("bloc"
                                                                          "k");
} /* block_detected() */

boost::optional<block_placer::placement_intent>
fcrw_bst_fsm::block_placement_intent(void) const {
  if (ekST_WAIT_FOR_BLOCK_PLACE == current_state()) {
    return boost::make_optional(m_block_place_fsm.placement_intent_calc());
  }
  return boost::none;
} /* block_placement_intent() */

double fcrw_bst_fsm::ct_bc_radius(void) const {
  /*
   * Calculate the side of a square inscribed in the "bounding circle"
   * containing the bounding box of the nearest construction target.
   */
  double side = std::max(mc_perception->nearest_ct()->bbr().x(),
                         mc_perception->nearest_ct()->bbr().y());

  /* Compute the radius of this circle from the square side length */
  return std::sqrt(std::pow(side, 2) * 2) / 2.0;
} /* ct_bc_radius() */

NS_END(fsm, silicon);
