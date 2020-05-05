/**
 * \file constructor_fsm.hpp
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

#ifndef INCLUDE_SILICON_FSM_CONSTRUCTOR_FSM_HPP_
#define INCLUDE_SILICON_FSM_CONSTRUCTOR_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <vector>

#include "cosm/spatial/fsm/util_hfsm.hpp"
#include "cosm/spatial/fsm/explore_for_goal_fsm.hpp"
#include "cosm/spatial/metrics/goal_acq_metrics.hpp"
#include "cosm/spatial/metrics/collision_metrics.hpp"
#include "cosm/fsm/block_transporter.hpp"

#include "silicon/silicon.hpp"
#include "silicon/fsm/builder_fsm.hpp"
#include "silicon/fsm/construction_transport_goal.hpp"
#include "silicon/lane_alloc/allocator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace silicon::controller::perception {
class builder_perception_subsystem;
} /* namespace silicon::controller */

NS_START(silicon, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class constructor_fsm
 * \ingroup fsm
 *
 * \brief The FSM for the most basic construction definition: each robot
 * executing this FSM:
 *
 * - Roams around randomly until it finds a block
 * - Brings the block to the nest
 * - Runs the \ref builder_fsm to traverse the structure, place the block on the
 *   structure, and then leave the structure.
 *
 * After these steps have been done, it signals it has completed its task.
 */
class constructor_fsm final : public csfsm::util_hfsm,
                              public rer::client<constructor_fsm>,
                              public csmetrics::goal_acq_metrics,
                              public cfsm::block_transporter<construction_transport_goal>,
                              public cta::taskable {
 public:
  constructor_fsm(const slaconfig::lane_alloc_config* allocator_config,
                  const scperception::builder_perception_subsystem* perception,
                  crfootbot::footbot_saa_subsystem* saa,
                  rmath::rng* rng);

  constructor_fsm(const constructor_fsm&) = delete;
  constructor_fsm& operator=(const constructor_fsm&) = delete;

  /* foraging collision metrics */
  bool in_collision_avoidance(void) const override RCSW_PURE;
  bool entered_collision_avoidance(void) const override RCSW_PURE;
  bool exited_collision_avoidance(void) const override RCSW_PURE;
  rtypes::timestep collision_avoidance_duration(void) const override RCSW_PURE;
  rmath::vector2z avoidance_loc2D(void) const override;
  rmath::vector3z avoidance_loc3D(void) const override;

  /* foraging goal acquisition metrics */
  csmetrics::goal_acq_metrics::goal_type acquisition_goal(void) const override RCSW_PURE;
  exp_status is_exploring_for_goal(void) const override RCSW_PURE;
  bool is_vectoring_to_goal(void) const override { return false; }
  bool goal_acquired(void) const override RCSW_PURE;
  rmath::vector2z acquisition_loc(void) const override;
  rtypes::type_uuid entity_acquired_id(void) const override RCSW_PURE;
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector2z, current_explore_loc, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector2z, current_vector_loc, const);

  /* block transportation */
  construction_transport_goal block_transport_goal(void) const override RCSW_PURE;

  /* taskable overrides */
  void task_execute(void) override;
  void task_start(cta::taskable_argument*) override {}
  bool task_finished(void) const override {
    return ekST_FINISHED == current_state();
  }
  bool task_running(void) const override {
    return ekST_FINISHED == current_state();
    }
  void task_reset(void) override { init(); }

  /**
   * \brief (Re)-initialize the FSM.
   */
  void init(void) override;

  /**
   * \brief Run the FSM in its current state, without injecting an event.
   */
  void run(void);

 private:
  enum fsm_states {
    ekST_START,

    /**
     * The robot is foraging for a block
     */
    ekST_FORAGE,

    /**
     * A block has been acquired, wait to get the block pickup signal.
     */
    ekST_WAIT_FOR_BLOCK_PICKUP,

    /**
     * The block is being returned to the nest; gradual alignment with the
     * chosen construction lane as the robot nears the nest.
     */
    ekST_TRANSPORT_TO_NEST,

    /**
     * The \ref builder_fsm is running.
     */
    ekST_BUILD,
    ekST_FINISHED,
    ekST_MAX_STATES
  };

  struct nest_transport_data final: public rpfsm::event_data {
    nest_transport_data(const csteer2D::ds::path_state& path_in,
                        const rmath::vector3d& ingress_in,
                        const rmath::vector3d& transport_start_in)
        : path(path_in),
          ingress(ingress_in),
          transport_start(transport_start_in) {}

    csteer2D::ds::path_state path;
    rmath::vector3d ingress{};
    rmath::vector3d transport_start{};
  };

  bool block_detected(void) const;

  std::vector<rmath::vector2d> calc_transport_path(
      const repr::construction_lane& lane) const;

  /* inherited states */
  HFSM_ENTRY_INHERIT_ND(csfsm::util_hfsm, entry_wait_for_signal);

  /* crw fsm states */
  HFSM_STATE_DECLARE_ND(constructor_fsm, start);
  HFSM_STATE_DECLARE_ND(constructor_fsm, forage);
  HFSM_STATE_DECLARE(constructor_fsm, wait_for_block_pickup,
                     rpfsm::event_data);
  HFSM_STATE_DECLARE(constructor_fsm, transport_to_nest, nest_transport_data);
  HFSM_ENTRY_DECLARE_ND(constructor_fsm, entry_transport_to_nest);
  HFSM_EXIT_DECLARE(constructor_fsm, exit_transport_to_nest);
  HFSM_STATE_DECLARE_ND(constructor_fsm, build);
  HFSM_STATE_DECLARE_ND(builder_fsm, finished);

  /**
   * \brief Defines the state map for the FSM.
   *
   * Note that the order of the states in the map MUST match the order of the
   * states in \enum fsm_states, or things will not work correctly.
   */
  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
  return (&mc_state_map[index]);
  }

  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ekST_MAX_STATES);

  /* clang-format off */
  const scperception::builder_perception_subsystem* mc_perception;

  lane_alloc::allocator                             m_allocator;
  repr::construction_lane                           m_lane{};
  csfsm::explore_for_goal_fsm                       m_forage_fsm;
  fsm::builder_fsm                                  m_build_fsm;
  /* clang-format on */
};

NS_END(fsm, silicon);

#endif /* INCLUDE_SILICON_FSM_CONSTRUCTOR_FSM_HPP_ */
