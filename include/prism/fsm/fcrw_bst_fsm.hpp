/**
 * \file fcrw_bst_fsm.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * This file is part of PRISM.
 *
 * PRISM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * PRISM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * PRISM.  If not, see <http://www.gnu.org/licenses/
 */

#ifndef INCLUDE_PRISM_FSM_FCRW_BST_FSM_HPP_
#define INCLUDE_PRISM_FSM_FCRW_BST_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <vector>

#include "cosm/fsm/block_transporter.hpp"
#include "cosm/spatial/fsm/explore_for_goal_fsm.hpp"
#include "cosm/spatial/metrics/goal_acq_metrics.hpp"

#include "prism/fsm/acquire_block_placement_site_fsm.hpp"
#include "prism/fsm/block_placer.hpp"
#include "prism/fsm/builder_util_fsm.hpp"
#include "prism/fsm/construction_transport_goal.hpp"
#include "prism/fsm/gmt_egress_fsm.hpp"
#include "prism/fsm/gmt_ingress_fsm.hpp"
#include "prism/lane_alloc/lane_allocator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace prism::repr {
class construction_lane;
} /* namespace prism::repr */

NS_START(prism, fsm);

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
 * - Runs the \ref builder_fsm to traverse the gmt, place the block on the
 *   gmt, and then leave the gmt, assuming there is a single
 *   structure present in the arena (Single Target).
 *
 * After these steps have been done, it signals it has completed its task.
 */
class fcrw_bst_fsm final
    : public builder_util_fsm,
      public rer::client<fcrw_bst_fsm>,
      public csmetrics::goal_acq_metrics,
      public cfsm::block_transporter<construction_transport_goal>,
      public block_placer {
 public:
  fcrw_bst_fsm(const placonfig::lane_alloc_config* allocator_config,
               const pcperception::builder_perception_subsystem* perception,
               csubsystem::saa_subsystemQ3D* saa,
               rmath::rng* rng);
  ~fcrw_bst_fsm(void) override;

  fcrw_bst_fsm(const fcrw_bst_fsm&) = delete;
  fcrw_bst_fsm& operator=(const fcrw_bst_fsm&) = delete;

  /* interference metrics */
  bool exp_interference(void) const override RCSW_PURE;
  bool entered_interference(void) const override RCSW_PURE;
  bool exited_interference(void) const override RCSW_PURE;
  rtypes::timestep interference_duration(void) const override RCSW_PURE;
  rmath::vector3z interference_loc3D(void) const override;

  /* goal acquisition metrics */
  csmetrics::goal_acq_metrics::goal_type
  acquisition_goal(void) const override RCSW_PURE;
  exp_status is_exploring_for_goal(void) const override RCSW_PURE;
  bool is_vectoring_to_goal(void) const override { return false; }
  bool goal_acquired(void) const override RCSW_PURE;
  rmath::vector3z acquisition_loc3D(void) const override;
  rtypes::type_uuid entity_acquired_id(void) const override RCSW_PURE;
  RCPPSW_WRAP_DECL_OVERRIDE(rmath::vector3z, explore_loc3D, const);
  RCPPSW_WRAP_DECL_OVERRIDE(rmath::vector3z, vector_loc3D, const);

  /* block transportation */
  construction_transport_goal block_transport_goal(void) const override RCSW_PURE;

  /* taskable overrides */
  void task_execute(void) override;
  void task_start(cta::taskable_argument*) override {}
  bool task_finished(void) const override {
    return ekST_FINISHED == current_state();
  }
  bool task_running(void) const override {
    return ekST_FINISHED != current_state() && ekST_START != current_state();
  }
  void task_reset(void) override { init(); }

  /* block placer overrides */
  boost::optional<repr::placement_intent>
  block_placement_intent(void) const override;

  const lane_alloc::lane_allocator* lane_allocator(void) const { return &m_allocator; }
  lane_alloc::lane_allocator* lane_allocator(void) { return &m_allocator; }

  /**
   * \brief (Re)-initialize the FSM.
   */
  void init(void) override;

 private:
  enum fsm_state {
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
     * The robot is moving from the block pickup location to the start of its
     * allocated construction lane.
     */
    ekST_SPCT_INGRESS,

    /**
     * A block has been placed on the structure; wait to get the placement
     * signal.
     */
    ekST_WAIT_FOR_BLOCK_PLACE,

    /**
     * The \ref acquire_block_placement_site_fsm is running.
     */
    ekST_SPCT_BUILD,

    /**
     * The \ref gmt_egress_fsm is running.
     */
    ekST_SPCT_EGRESS,
    ekST_FINISHED,
    ekST_MAX_STATES
  };

  bool block_detected(void) const;

  /* inherited states */
  RCPPSW_HFSM_ENTRY_INHERIT_ND(csfsm::util_hfsm, entry_wait_for_signal);

  /* crw fsm states */
  RCPPSW_HFSM_STATE_DECLARE_ND(fcrw_bst_fsm, start);
  RCPPSW_HFSM_STATE_DECLARE_ND(fcrw_bst_fsm, forage);
  RCPPSW_HFSM_STATE_DECLARE(fcrw_bst_fsm,
                            wait_for_block_pickup,
                            rpfsm::event_data);
  RCPPSW_HFSM_STATE_DECLARE(fcrw_bst_fsm,
                            wait_for_block_place,
                            rpfsm::event_data);
  RCPPSW_HFSM_STATE_DECLARE_ND(fcrw_bst_fsm, gmt_ingress);
  RCPPSW_HFSM_STATE_DECLARE_ND(fcrw_bst_fsm, structure_build);
  RCPPSW_HFSM_STATE_DECLARE_ND(fcrw_bst_fsm, gmt_egress);
  RCPPSW_HFSM_STATE_DECLARE_ND(fcrw_bst_fsm, finished);

  /**
   * \brief Defines the state map for the FSM.
   *
   * Note that the order of the states in the map MUST match the order of the
   * states in \enum fsm_states, or things will not work correctly.
   */
  RCPPSW_HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
    return (&mc_state_map[index]);
  }

  RCPPSW_HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ekST_MAX_STATES);

  /* clang-format off */
  lane_alloc::lane_allocator                        m_allocator;
  std::unique_ptr<prepr::construction_lane>         m_allocated_lane;
  csfsm::explore_for_goal_fsm                       m_forage_fsm;
  acquire_block_placement_site_fsm                  m_block_place_fsm;
  gmt_ingress_fsm                             m_gmt_ingress_fsm;
  gmt_egress_fsm                              m_gmt_egress_fsm;
  /* clang-format on */
};

NS_END(fsm, prism);

#endif /* INCLUDE_PRISM_FSM_FCRW_BST_FSM_HPP_ */
