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
acquire_block_placement_site_fsm::acquire_block_placement_site_fsm(const scperception::builder_perception_subsystem* perception,
                                 crfootbot::footbot_saa_subsystem* saa,
                                 rmath::rng* rng)
    : builder_util_fsm(perception, saa, rng, fsm_state::ekST_MAX_STATES),
      ER_CLIENT_INIT("silicon.fsm.acquire_block_placement_site"),
      HFSM_CONSTRUCT_STATE(wait_for_robot, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(acquire_frontier_set, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(acquire_placement_loc, hfsm::top_state()),
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
          HFSM_STATE_MAP_ENTRY_EX(&finished)) {}

acquire_block_placement_site_fsm::~acquire_block_placement_site_fsm(void) = default;

/*******************************************************************************
 * States
 ******************************************************************************/
HFSM_STATE_DEFINE_ND(acquire_block_placement_site_fsm, start) {
  return rpfsm::event_signal::ekHANDLED;
}

HFSM_ENTRY_DEFINE_ND(acquire_block_placement_site_fsm,
                     entry_acquire_frontier_set) {
  saa()->sensing()->sensor<chal::sensors::colored_blob_camera_sensor>()->enable();
}

HFSM_STATE_DEFINE_ND(acquire_block_placement_site_fsm,
                     acquire_frontier_set) {
  ER_ASSERT(lane_alignment_verify_pos(lane()->ingress().to_2D(),
                                      -lane()->orientation()),
            "Bad alignment (position) during frontier set acqusition");
  ER_ASSERT(lane_alignment_verify_azimuth(-lane()->orientation()),
            "Bad alignment (orientation) during frontier set acquisition");

  /*
   * A robot in front of us is too close (either on structure or when we are in
   * the nest moving towards the structure)--wait for it to move before
   * continuing.
   */
  if (robot_trajectory_proximity()) {
    internal_event(fsm_state::ekST_WAIT_FOR_ROBOT,
                   std::make_unique<robot_wait_data>(robot_proximity_type::ekTRAJECTORY));
    return rpfsm::event_signal::ekHANDLED;
  }
  /* No robots in front, but not in nest yet */
  else if (nullptr == perception()->los()) {
    ER_TRACE("Robot=%s/%s: No robots detected, not in nest",
             sensing()->rpos2D().to_str().c_str(),
             sensing()->dpos2D().to_str().c_str());
    saa()->steer_force2D().accum(saa()->steer_force2D().path_following(m_path.get()));
    return rpfsm::event_signal::ekHANDLED;
  } else {
    /* somewhere in nest or on structure */
    auto acq = frontier_set_acquisition();
    ER_TRACE("Robot=%s/%s: On structure, acq=%d",
             sensing()->rpos2D().to_str().c_str(),
             sensing()->dpos2D().to_str().c_str(),
             rcppsw::as_underlying(acq));

    if (stygmergic_configuration::ekNONE == acq) {
      /* no robots in front, but not there yet */
      saa()->steer_force2D().accum(saa()->steer_force2D().path_following(m_path.get()));
    } else { /* arrived! */
      m_path = std::make_unique<csteer2D::ds::path_state>(calc_placement_path(acq));

      internal_event(fsm_state::ekST_ACQUIRE_PLACEMENT_LOC);
    }
    return rpfsm::event_signal::ekHANDLED;
  }
}

HFSM_STATE_DEFINE_ND(acquire_block_placement_site_fsm, acquire_placement_loc) {
  /*
   * We don't check for trajectory proximity to other robots while acquiring the
   * placement location, because when we get to this part, we are guaranteed to
   * be the only robot in this state in THIS construction lane. We DO, however,
   * need to check for manhattan proximity in order to avoid potential
   * deadlocks.
   */
  if (robot_manhattan_proximity()) {
    internal_event(fsm_state::ekST_WAIT_FOR_ROBOT,
                   std::make_unique<robot_wait_data>(robot_proximity_type::ekMANHATTAN));
  } else {
    auto force = saa()->steer_force2D().path_following(m_path.get());
    if (force.is_pd()) {
      saa()->steer_force2D().accum(force);
    } else {
      m_path.reset();
      internal_event(fsm_state::ekST_FINISHED);
    }
  }
  return rpfsm::event_signal::ekHANDLED;
}

HFSM_STATE_DEFINE_ND(acquire_block_placement_site_fsm, finished) {
  if (fsm_state::ekST_FINISHED != last_state()) {
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
void acquire_block_placement_site_fsm::task_start(cta::taskable_argument* c_arg) {
  static const uint8_t kTRANSITIONS[] = {
    fsm_state::ekST_ACQUIRE_FRONTIER_SET, /* start */
    rpfsm::event_signal::ekFATAL, /* acquire_frontier_set */
    rpfsm::event_signal::ekFATAL, /* wait_for_robot */
    rpfsm::event_signal::ekFATAL, /* acquire placement loc */
    fsm_state::ekST_ACQUIRE_FRONTIER_SET,  /* finished */
  };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, fsm_state::ekST_MAX_STATES);

  auto* const a = dynamic_cast<repr::construction_lane*>(c_arg);
  ER_ASSERT(nullptr != a, "Bad construction lane argument");
  lane(std::move(*a));

  m_path = std::make_unique<csteer2D::ds::path_state>(calc_frontier_set_path());
  external_event(kTRANSITIONS[current_state()]);
} /* task_start() */

void acquire_block_placement_site_fsm::task_execute(void) {
  inject_event(rpfsm::event_signal::ekRUN, rpfsm::event_type::ekNORMAL);
} /* task_execute() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void acquire_block_placement_site_fsm::init(void) {
  actuation()->reset();
  util_hfsm::init();
} /* init() */

stygmergic_configuration acquire_block_placement_site_fsm::frontier_set_acquisition(void) const {
  auto pos = saa()->sensing()->dpos3D();
  rmath::vector3z ingress_abs;
  rmath::vector3z egress_abs;

  /*
   * \todo Right now this assumes cube blocks, and is only correct for
   * such. This will be fixed once the most basic construction has been
   * validated.
   */
  if (rmath::radians::kZERO == lane()->orientation()) {
    ingress_abs = pos - rmath::vector3z::X * 2;
    egress_abs = pos - rmath::vector3z::X * 2 + rmath::vector3z::Y;
  } else if (rmath::radians::kPI_OVER_TWO == lane()->orientation()) {
    ingress_abs = pos - rmath::vector3z::Y * 2;
    egress_abs = pos - rmath::vector3z::Y * 2 - rmath::vector3z::X;
  } else {
    ER_FATAL_SENTINEL("Bad lane orientation '%s'",
                      lane()->orientation().to_str().c_str());
  }
  auto origin = perception()->los()->abs_ll();
  bool ingress_has_block = perception()->los()->contains_loc(ingress_abs - origin) &&
                           perception()->los()->access((ingress_abs - origin)).state_has_block();
  bool egress_has_block = perception()->los()->contains_loc(egress_abs - origin) &&
                          perception()->los()->access((egress_abs - origin)).state_has_block();

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

std::vector<rmath::vector2d> acquire_block_placement_site_fsm::calc_placement_path(
    const stygmergic_configuration& acq) const {
  std::vector<rmath::vector2d> path;
  auto pos = saa()->sensing()->dpos2D();

  /*
   * \todo Right now this assumes cube blocks, and is only correct for
   * such. This will be fixed once the most basic construction has been
   * validated.
   */
  if (rmath::radians::kZERO == lane()->orientation()) {
    if (stygmergic_configuration::ekLANE_GAP_INGRESS == acq) {
      path.push_back(rmath::zvec2dvec({pos.x() - 1, pos.y()},
                                      perception()->arena_resolution().v()));
    } else if (stygmergic_configuration::ekLANE_GAP_EGRESS == acq) {
      path.push_back(rmath::zvec2dvec({pos.x() - 1, pos.y()},
                                      perception()->arena_resolution().v()));
      path.push_back(rmath::zvec2dvec({pos.x() - 1, pos.y() + 1},
                                      perception()->arena_resolution().v()));
    }
  } else if (rmath::radians::kPI_OVER_TWO == lane()->orientation()) {
    if (stygmergic_configuration::ekLANE_GAP_INGRESS == acq) {
      path.push_back(rmath::zvec2dvec({pos.x(), pos.y() - 1},
                                      perception()->arena_resolution().v()));
    } else if (stygmergic_configuration::ekLANE_GAP_EGRESS == acq) {
      path.push_back(rmath::zvec2dvec({pos.x(), pos.y() - 1},
                                      perception()->arena_resolution().v()));
      path.push_back(rmath::zvec2dvec({pos.x() - 1, pos.y()},
                                      perception()->arena_resolution().v()));
    }
  }
  return path;
} /* calc_placement_path() */

std::vector<rmath::vector2d> acquire_block_placement_site_fsm::calc_frontier_set_path(void) {
  std::vector<rmath::vector2d> path;
  /*
   * Simple path to go from the robots current position to the back of the
   * structure (end will never be reached). This is done so a reference to the
   * structure being created is not needed and the robot can be more
   * stygmergic.
   */
  if (rmath::radians::kZERO == lane()->orientation()) {

    path.push_back({0.0, lane()->ingress().y()});
  } else if (rmath::radians::kPI_OVER_TWO == lane()->orientation()) {
    /*
     * Simple path to go from the robots current position to the back of the
     * structure (end will never be reached). This is done so a reference to the
     * structure being created is not needed and the robot can be more
     * stygmergic.
     */
    path.push_back({lane()->ingress().x(), 0.0});
  }
  return path;
} /* calc_frontier_set_path() */

rmath::vector3z acquire_block_placement_site_fsm::calc_placement_cell(void) const {
  ER_ASSERT(fsm_state::ekST_FINISHED == current_state(),
            "Placement cell can only be calculated once the FSM finishes");
  auto pos = saa()->sensing()->dpos3D();
  /*
   * \todo Right now this assumes cube blocks, and is only correct for
   * such. This will be fixed once the most basic construction has been
   * validated.
   */
  if (rmath::radians::kZERO == lane()->orientation()) {
    return {pos.x() - 1, pos.y(), pos.z()};
  } else if (rmath::radians::kPI_OVER_TWO == lane()->orientation()) {
    return {pos.x(), pos.y() - 1, pos.z()};
  } else {
    ER_FATAL_SENTINEL("Bad lane orientation '%s'",
                      lane()->orientation().to_str().c_str());
  }
} /* calc_placement_cell() */

NS_END(fsm, silicon);
