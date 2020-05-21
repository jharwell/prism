/**
 * \file fcrw_bst_fsm.hpp
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

#ifndef INCLUDE_SILICON_FSM_FCRW_BST_FSM_HPP_
#define INCLUDE_SILICON_FSM_FCRW_BST_FSM_HPP_

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
#include "silicon/fsm/acquire_block_placement_site_fsm.hpp"
#include "silicon/fsm/structure_egress_fsm.hpp"
#include "silicon/fsm/construction_transport_goal.hpp"
#include "silicon/lane_alloc/allocator.hpp"
#include "silicon/fsm/block_placer.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class fcrw_bst_fsm
 * \ingroup fsm
 *
 * \brief The FSM for CRW Foraging (FCRW), Single Target Building (BST). Each
 * robot executing this FSM:
 *
 * - Roams around randomly until it finds a block (Foraging CRW)
 * - Brings the block to the nest
 * - Runs the \ref builder_fsm to traverse the structure, place the block on the
 *   structure, and then leave the structure, assuming there is a single
 *   structure present in the arena (Single Target).
 *
 * After these steps have been done, it signals it has completed its task.
 */
class fcrw_bst_fsm final : public csfsm::util_hfsm,
                           public rer::client<fcrw_bst_fsm>,
                           public csmetrics::goal_acq_metrics,
                           public cfsm::block_transporter<construction_transport_goal>,
                           public cta::taskable,
                           public block_placer {
 public:
  fcrw_bst_fsm(const slaconfig::lane_alloc_config* allocator_config,
                  const scperception::builder_perception_subsystem* perception,
                  crfootbot::footbot_saa_subsystem* saa,
                  rmath::rng* rng);
  ~fcrw_bst_fsm(void) override;

  fcrw_bst_fsm(const fcrw_bst_fsm&) = delete;
  fcrw_bst_fsm& operator=(const fcrw_bst_fsm&) = delete;

  /* foraging collision metrics */
  bool in_collision_avoidance(void) const override RCSW_PURE;
  bool entered_collision_avoidance(void) const override RCSW_PURE;
  bool exited_collision_avoidance(void) const override RCSW_PURE;
  rtypes::timestep collision_avoidance_duration(void) const override RCSW_PURE;
  rmath::vector2z avoidance_loc2D(void) const override;
  rmath::vector3z avoidance_loc3D(void) const override;

  /* goal acquisition metrics */
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

  /* block placer overrides */
  boost::optional<block_placer::placement_intent> block_placement_intent(void) const override;

  const lane_alloc::allocator* lane_allocator(void) const {
    return &m_allocator;
  }

  /**
   * \brief (Re)-initialize the FSM.
   */
  void init(void) override;

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
     * The robot is moving from the block pickup location to the face of the
     * construction target which has the ingress/egress lanes.
     */
    ekST_CT_APPROACH,

    /**
     * The block is being returned to the nest; gradual alignment with the
     * chosen construction lane as the robot nears the nest.
     */
    ekST_CT_ENTRY,

    /**
     * A block has been placed on the structure, wait to get the placement
     * signal.
     */
    ekST_WAIT_FOR_BLOCK_PLACE,

    /**
     * The \ref acquire_block_placement_site_fsm is running.
     */
    ekST_STRUCTURE_BUILD,

    /**
     * The \ref structure_egress_fsm is running.
     */
    ekST_STRUCTURE_EGRESS,
    ekST_FINISHED,
    ekST_MAX_STATES
  };

  struct ct_ingress_data final: public rpfsm::event_data {
    ct_ingress_data(const csteer2D::ds::path_state& path_in,
                        const rmath::vector3d& ingress_in)
        : path(path_in),
          ingress(ingress_in) {}

    csteer2D::ds::path_state path;
    rmath::vector3d ingress;
  };

  static constexpr const double kCT_TRANSPORT_BC_DIST_MIN = 5.0;
  static constexpr const double kCT_TRANSPORT_LANE_PROX_CRITICAL_DIST = 3.0;

  bool block_detected(void) const;

  double ct_bc_radius(void) const;

  /* inherited states */
  HFSM_ENTRY_INHERIT_ND(csfsm::util_hfsm, entry_wait_for_signal);

  /* crw fsm states */
  HFSM_STATE_DECLARE_ND(fcrw_bst_fsm, start);
  HFSM_STATE_DECLARE_ND(fcrw_bst_fsm, forage);
  HFSM_STATE_DECLARE(fcrw_bst_fsm, wait_for_block_pickup, rpfsm::event_data);
  HFSM_STATE_DECLARE(fcrw_bst_fsm, wait_for_block_place, rpfsm::event_data);
  HFSM_STATE_DECLARE_ND(fcrw_bst_fsm, ct_approach);
  HFSM_STATE_DECLARE(fcrw_bst_fsm, ct_entry, ct_ingress_data);
  HFSM_ENTRY_DECLARE_ND(fcrw_bst_fsm, entry_ct_approach);
  HFSM_EXIT_DECLARE(fcrw_bst_fsm, exit_ct_entry);
  HFSM_STATE_DECLARE_ND(fcrw_bst_fsm, structure_build);
  HFSM_STATE_DECLARE_ND(fcrw_bst_fsm, structure_egress);
  HFSM_STATE_DECLARE_ND(fcrw_bst_fsm, finished);

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
  acquire_block_placement_site_fsm                  m_block_place_fsm;
  structure_egress_fsm                              m_structure_egress_fsm;
  /* clang-format on */
};

NS_END(fsm, silicon);

#endif /* INCLUDE_SILICON_FSM_FCRW_BST_FSM_HPP_ */
