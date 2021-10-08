/**
 * \file acquire_block_placement_site_fsm.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>
#include <memory>
#include <vector>

#include "cosm/steer2D/ds/path_state.hpp"

#include "prism/fsm/block_placer.hpp"
#include "prism/fsm/builder_util_fsm.hpp"
#include "prism/fsm/calculators/lane_alignment.hpp"
#include "prism/repr/fs_configuration.hpp"
#include "prism/fsm/calculators/fs_acq/base_strategy.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class acquire_block_placement_site_fsm
 * \ingroup fsm
 *
 * \brief The FSM for navigating a chosen construction lane once the robot has
 * entered the construction area to place the block it is carrying at an
 * appropriate place on the structure. After this has been done, the FSM signals
 * that it has completed its task.
 */
class acquire_block_placement_site_fsm
    : public rer::client<acquire_block_placement_site_fsm>,
      public builder_util_fsm {
 public:
  acquire_block_placement_site_fsm(const pfsm::fsm_params* params,
                                   rmath::rng* rng);

  ~acquire_block_placement_site_fsm(void) override;

  /* not copy constructible or copy assignable by default */
  acquire_block_placement_site_fsm(const acquire_block_placement_site_fsm&) =
      delete;
  acquire_block_placement_site_fsm&
  operator=(const acquire_block_placement_site_fsm&) = delete;

  /* taskable overrides */
  bool task_running(void) const override {
    return current_state() != fsm_state::ekST_START &&
           current_state() != fsm_state::ekST_FINISHED;
  }
  void task_execute(void) override;
  void task_start(cta::taskable_argument* c_arg) override;
  bool task_finished(void) const override {
    return current_state() == fsm_state::ekST_FINISHED;
  }
  void task_reset(void) override;

  /**
   * \brief Once the robot has reached the end of the path from which it should
   * be able to place its block, calculate (1) the cell within the construction
   * target that the robot wants to place its block in, and (2) the block's
   * orientation within the chosen cell.
   *
   * Can only be called if the FSM is in the finished state.
   */
  repr::placement_intent placement_intent_calc(void) const;

 private:
  enum fsm_state {
    ekST_START,
    /**
     * Moving along the ingress lane until the robot nears the end and is able
     * to see via its LOS where its blocks should be placed.
     */
    ekST_ACQUIRE_FRONTIER_SET,

    /**
     * The robot is waiting for the robot ahead of it to continue moving so it
     * can continue along the ingress/egress lane (robots always leave space
     * between them and the robot in front of them to avoid deadlocks).
     */
    ekST_WAIT_FOR_ROBOT,

    /**
     * The robot has reached the frontier set and is moving to the cell from
     * which it can place the block it is holding in an appropriate location.
     */
    ekST_ACQUIRE_PLACEMENT_LOC,

    /**
     * The robot has left the structure and returned to the arena.
     */
    ekST_FINISHED,

    ekST_MAX_STATES
  };

  /* inherited states */
  RCPPSW_HFSM_STATE_INHERIT(builder_util_fsm, wait_for_robot, robot_wait_data);
  RCPPSW_HFSM_ENTRY_INHERIT_ND(builder_util_fsm, entry_wait_for_robot);
  RCPPSW_HFSM_EXIT_INHERIT(builder_util_fsm, exit_wait_for_robot);

  /* FSM states */
  RCPPSW_HFSM_STATE_DECLARE_ND(acquire_block_placement_site_fsm, start);
  RCPPSW_HFSM_STATE_DECLARE_ND(acquire_block_placement_site_fsm,
                               acquire_frontier_set);

  RCPPSW_HFSM_ENTRY_DECLARE_ND(acquire_block_placement_site_fsm,
                               entry_acquire_frontier_set);

  RCPPSW_HFSM_ENTRY_DECLARE_ND(acquire_block_placement_site_fsm,
                               entry_acquire_placement_loc);
  RCPPSW_HFSM_STATE_DECLARE_ND(acquire_block_placement_site_fsm,
                               acquire_placement_loc);

  RCPPSW_HFSM_STATE_DECLARE_ND(acquire_block_placement_site_fsm, finished);


  RCPPSW_HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
    return &mc_state_map[index];
  }

  RCPPSW_HFSM_DECLARE_STATE_MAP(state_map_ex,
                                mc_state_map,
                                fsm_state::ekST_MAX_STATES);

  /* clang-format off */
  std::unique_ptr<csteer2D::ds::path_state>             m_ingress_path{nullptr};
  std::unique_ptr<csteer2D::ds::path_state>             m_site_path{nullptr};
  std::unique_ptr<pfcalculators::fs_acq::base_strategy> m_fs_acq_strat;
  pfcalculators::lane_alignment                         m_alignment_calc;
  prepr::fs_acq_result                                  m_site_fs{};
  /* clang-format on */
};

NS_END(fsm, prism);

