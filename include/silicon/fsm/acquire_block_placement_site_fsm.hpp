/**
 * \file acquire_block_placement_site_fsm.hpp
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

#ifndef INCLUDE_SILICON_FSM_ACQUIRE_BLOCK_PLACEMENT_SITE_FSM_HPP_
#define INCLUDE_SILICON_FSM_ACQUIRE_BLOCK_PLACEMENT_SITE_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <memory>

#include <boost/optional.hpp>

#include "cosm/steer2D/ds/path_state.hpp"

#include "silicon/fsm/builder_util_fsm.hpp"
#include "silicon/fsm/stygmergic_configuration.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, fsm);

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
class acquire_block_placement_site_fsm : public builder_util_fsm,
                                         public rer::client<acquire_block_placement_site_fsm> {
 public:
  acquire_block_placement_site_fsm(const scperception::builder_perception_subsystem* perception,
              crfootbot::footbot_saa_subsystem* saa,
              rmath::rng* rng);

  ~acquire_block_placement_site_fsm(void) override;

  /* not copy constructible or copy assignable by default */
  acquire_block_placement_site_fsm(const acquire_block_placement_site_fsm&) = delete;
  acquire_block_placement_site_fsm& operator=(const acquire_block_placement_site_fsm&) = delete;

  /* taskable overrides */
  void task_reset(void) override { init(); }
  bool task_running(void) const override {
    return current_state() != fsm_state::ekST_START && current_state() != fsm_state::ekST_FINISHED;
  }
  void task_execute(void) override;
  void task_start(cta::taskable_argument* c_arg) override;
  bool task_finished(void) const override {
    return current_state() == fsm_state::ekST_FINISHED;
  }

  /* HFSM overrides */
  void init(void) override;

  /**
   * \brief Once the robot has reached the end of the path from which it should
   * be able to place its block, calculate the cell that the robot wants to
   * place its block in.
   *
   * Can only be called if the FSM is in the finished state.
   */
  rmath::vector3z calc_placement_cell(void) const;

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
  HFSM_STATE_INHERIT(builder_util_fsm, wait_for_robot, const robot_wait_data);

  /* FSM states */
  HFSM_STATE_DECLARE_ND(acquire_block_placement_site_fsm, start);
  HFSM_STATE_DECLARE_ND(acquire_block_placement_site_fsm,
                        acquire_frontier_set);

  HFSM_ENTRY_DECLARE_ND(acquire_block_placement_site_fsm,
                        entry_acquire_frontier_set);
  HFSM_STATE_DECLARE_ND(acquire_block_placement_site_fsm,
                        acquire_placement_loc);

  HFSM_STATE_DECLARE_ND(acquire_block_placement_site_fsm, finished);

  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
    return &mc_state_map[index];
  }

  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, fsm_state::ekST_MAX_STATES);

  /**
   * \brief Retur the stygmergic configuration the robot has encountered when it
   * has reached the frontier set, and an empty configuration otherwise.
   */
  stygmergic_configuration frontier_set_acquisition(void) const;

  /**
   * \brief Calculate the path from the robot's current position to the frontier
   * set.
   */
  std::vector<rmath::vector2d> calc_frontier_set_path(void);

  /**
   * \brief Calculate the path from the location at which the robot has reached
   * the frontier set to an appropriate cell from which to place its block,
   * given the current shape of the frontier set it encounters.
   */
  std::vector<rmath::vector2d> calc_placement_path(
      const stygmergic_configuration& acq) const;

  /* clang-format off */
  std::unique_ptr<csteer2D::ds::path_state> m_path{nullptr};
  /* clang-format on */
};

NS_END(fsm, silicon);

#endif /* INCLUDE_SILICON_FSM_ACQUIRE_BLOCK_PLACEMENT_SITE_FSM_HPP_ */
