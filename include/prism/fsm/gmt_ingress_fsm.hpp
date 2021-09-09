/**
 * \file gmt_ingress_fsm.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_PRISM_FSM_SPCT_INGRESS_FSM_HPP_
#define INCLUDE_PRISM_FSM_SPCT_INGRESS_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "cosm/steer2D/ds/path_state.hpp"

#include "prism/fsm/builder_util_fsm.hpp"

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
 * \class gmt_ingress_fsm
 * \ingroup fsm
 *
 * \brief This FSM runs after a robot has picked up a block. Robots executing
 * this FSM:
 *
 * - Brings the block to the nest by traversing the circumference of the
 *   construction zone.
 * - Allocate a construction lane and acquire the lane's ingress point.
 *
 * After these steps have been done, it signals it has completed its task.
 */
class gmt_ingress_fsm final : public builder_util_fsm,
                                    public rer::client<gmt_ingress_fsm> {
 public:
  gmt_ingress_fsm(
      const pcperception::builder_perception_subsystem* perception,
      csubsystem::saa_subsystemQ3D* saa,
      rmath::rng* rng);
  ~gmt_ingress_fsm(void) override;

  gmt_ingress_fsm(const gmt_ingress_fsm&) = delete;
  gmt_ingress_fsm& operator=(const gmt_ingress_fsm&) = delete;

  /* taskable overrides */
  void task_execute(void) override;
  void task_start(cta::taskable_argument* c_arg) override;
  bool task_finished(void) const override {
    return ekST_FINISHED == current_state();
  }
  bool task_running(void) const override {
    return ekST_FINISHED != current_state() && ekST_START != current_state();
  }
  void task_reset(void) override { init(); }

  /**
   * \brief (Re)-initialize the FSM.
   */
  void init(void) override;

 private:
  enum fsm_state {
    ekST_START,

    /**
     * The robot is moving from the block pickup location to the face of the
     * construction target which has the ingress/egress lanes.
     */
    ekST_CT_APPROACH,

    /**
     * The robot is waiting for the robot ahead of it to continue moving so it
     * can continue along the ingress lane (robots always leave space between
     * them and the robot in front of them to avoid deadlocks).
     */
    ekST_WAIT_FOR_ROBOT,

    /**
     * The robot is approximately aligned with its allocated construction lane;
     * it continues to move to the lane ingress point.
     */
    ekST_CT_ENTRY,

    ekST_FINISHED,
    ekST_MAX_STATES
  };

  /**
   * When transporting a block to the construction site via polar forces, this
   * is the extra distance adding to the diameter of the circle inscribed by the
   * bounding box for the structure which robots use to maintain sufficient
   * distance from the construction zone while transporting.
   */
  static constexpr const double kCT_APPROACH_BB_CIRCLE_DIAMETER_PAD = 5.0;

  /**
   * \brief Calculate the radius of the smallest circle in which the bounding
   * box containing the current construction target can be inscribed for use in
   * calculating the approach vector to the allocated \ref
   * prepr::construction_lane.
   */
  double ct_bb_circle_radius(void) const;

  double path_factor_calc(void) const;

  /* inherited states */
  RCPPSW_HFSM_ENTRY_INHERIT_ND(csfsm::util_hfsm, entry_wait_for_signal);
  RCPPSW_HFSM_STATE_INHERIT(builder_util_fsm, wait_for_robot, robot_wait_data);

  /* structure ingress states */
  RCPPSW_HFSM_STATE_DECLARE_ND(gmt_ingress_fsm, start);
  RCPPSW_HFSM_STATE_DECLARE_ND(gmt_ingress_fsm, ct_approach);
  RCPPSW_HFSM_STATE_DECLARE_ND(gmt_ingress_fsm, ct_entry);
  RCPPSW_HFSM_STATE_DECLARE_ND(gmt_ingress_fsm, finished);

  RCPPSW_HFSM_ENTRY_DECLARE_ND(gmt_ingress_fsm, entry_wait_for_robot);
  RCPPSW_HFSM_EXIT_DECLARE(gmt_ingress_fsm, exit_wait_for_robot);

  RCPPSW_HFSM_ENTRY_DECLARE_ND(gmt_ingress_fsm, entry_ct_approach);
  RCPPSW_HFSM_EXIT_DECLARE(gmt_ingress_fsm, exit_ct_entry);

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
  std::unique_ptr<csteer2D::ds::path_state> m_ingress_state{nullptr};
  double                                    m_ct_approach_polar_sign{-1};
  /* clang-format on */

 public:
};

NS_END(fsm, prism);

#endif /* INCLUDE_PRISM_FSM_SPCT_INGRESS_FSM_HPP_ */
