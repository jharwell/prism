/**
 * \file gmt_egress_fsm.hpp
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
#include <memory>
#include <vector>

#include "cosm/steer2D/ds/path_state.hpp"

#include "prism/fsm/builder_util_fsm.hpp"
#include "prism/fsm/calculators/lane_alignment.hpp"
#include "prism/fsm/fsm_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class gmt_egress_fsm
 * \ingroup fsm
 *
 * \brief The FSM for exiting the in-progress structure once a robot has placed
 * its block.
 */
class gmt_egress_fsm : public builder_util_fsm,
                             public rer::client<gmt_egress_fsm> {
 public:
  gmt_egress_fsm(const pfsm::fsm_params* params,
                 rmath::rng* rng);

  ~gmt_egress_fsm(void) override = default;

  /* not copy constructible or copy assignable by default */
  gmt_egress_fsm(const gmt_egress_fsm&) = delete;
  gmt_egress_fsm& operator=(const gmt_egress_fsm&) = delete;

  /* taskable overrides */
  void task_reset(void) override { init(); }
  bool task_running(void) const override {
    return current_state() != ekST_START && current_state() != ekST_FINISHED;
  }
  void task_execute(void) override;
  void task_start(cta::taskable_argument* c_arg) override;
  bool task_finished(void) const override {
    return current_state() == ekST_FINISHED;
  }

  void init(void) override;

 private:
  enum fsm_state {
    ekST_START,
    /**
     * The robot has placed its block and is moving from the cell it placed the
     * block FROM to the egress lane (it may already be there).
     */
    ekST_ACQUIRE_EGRESS_LANE,

    /**
     * The robot is leaving the structure via the egress lane.
     */
    ekST_SPCT_EGRESS,

    /**
     * The robot is waiting for the robot in front of it to move so it can
     * continue.
     */
    ekST_WAIT_FOR_ROBOT,

    /**
     * The robot has left the structure and returned to the arena.
     */
    ekST_FINISHED,

    ekST_MAX_STATES
  };

  /* inherited states */
  RCPPSW_HFSM_ENTRY_INHERIT_ND(util_hfsm, entry_wait_for_signal);
  RCPPSW_HFSM_STATE_INHERIT(builder_util_fsm, wait_for_robot, robot_wait_data);
  RCPPSW_HFSM_ENTRY_INHERIT_ND(builder_util_fsm, entry_wait_for_robot);
  RCPPSW_HFSM_EXIT_INHERIT(builder_util_fsm, exit_wait_for_robot);

  /* builder states */
  RCPPSW_HFSM_STATE_DECLARE_ND(gmt_egress_fsm, start);
  RCPPSW_HFSM_STATE_DECLARE_ND(gmt_egress_fsm, acquire_egress_lane);
  RCPPSW_HFSM_ENTRY_DECLARE_ND(gmt_egress_fsm, entry_acquire_egress_lane);
  RCPPSW_HFSM_STATE_DECLARE_ND(gmt_egress_fsm, gmt_egress);
  RCPPSW_HFSM_ENTRY_DECLARE_ND(gmt_egress_fsm, entry_finished);
  RCPPSW_HFSM_ENTRY_DECLARE_ND(gmt_egress_fsm, entry_gmt_egress);

  RCPPSW_HFSM_STATE_DECLARE_ND(gmt_egress_fsm, finished);

  RCPPSW_HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
    return &mc_state_map[index];
  }

  RCPPSW_HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ekST_MAX_STATES);

  /* clang-format off */
  calculators::lane_alignment               m_alignment_calc;
  std::unique_ptr<csteer2D::ds::path_state> m_egress_state{nullptr};
  /* clang-format on */
};

NS_END(fsm, prism);

