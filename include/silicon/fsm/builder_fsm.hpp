/**
 * \file builder_fsm.hpp
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

#ifndef INCLUDE_SILICON_FSM_BUILDER_FSM_HPP_
#define INCLUDE_SILICON_FSM_BUILDER_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/er/client.hpp"

#include "cosm/robots/footbot/footbot_subsystem_fwd.hpp"
#include "cosm/ta/taskable.hpp"
#include "cosm/spatial/fsm/util_hfsm.hpp"
#include "cosm/steer2D/ds/path_state.hpp"

#include "silicon/silicon.hpp"
#include "silicon/repr/construction_lane.hpp"
#include "silicon/fsm/stygmergic_configuration.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace silicon::controller::perception {
class builder_perception_subsystem;
} /* namespace silicon::perception */

NS_START(silicon, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class builder_fsm
 * \ingroup fsm
 *
 * \brief The FSM for navigating a chosen construction lane once the robot has
 * entered the construction area to place the block it is carrying at an
 * appropriate place on the structure.
 *
 * Once the block it is carrying has been placed and the robot has exited the
 * structure returned to the 2D arena, it signals that it has completed its
 * task.
 */
class builder_fsm : public csfsm::util_hfsm,
                    public cta::taskable,
                    public rer::client<builder_fsm> {
 public:
  builder_fsm(const scperception::builder_perception_subsystem* perception,
              crfootbot::footbot_saa_subsystem* saa,
              rmath::rng* rng);

  ~builder_fsm(void) override = default;

  /* not copy constructible or copy assignable by default */
  builder_fsm(const builder_fsm&) = delete;
  builder_fsm& operator=(const builder_fsm&) = delete;

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

  /* HFSM overrides */
  void init(void) override;

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
     * Wait to get the block placement signal.
     */
    ekST_WAIT_FOR_BLOCK_PLACE,

    /**
     * The robot has placed its block and is moving from the cell it placed the
     * block FROM to the egress lane (it may already be there).
     */
    ekST_ACQUIRE_EGRESS_LANE,

    /**
     * The robot is leaving the structure via the egress lane.
     */
    ekST_STRUCTURE_EGRESS,

    /**
     * The robot has left the structure and returned to the arena.
     */
    ekST_FINISHED,

    ekST_MAX_STATES
  };

  enum class robot_proximity_type {
    ekTRAJECTORY,
    ekMANHATTAN
  };

  struct robot_wait_data final : public rpfsm::event_data {
    explicit robot_wait_data(robot_proximity_type in) : prox_type(in) {}
    robot_proximity_type prox_type;
  };

  /**
   * How close the robot needs to be to the vectors representing the
   * ingress/egress lanes in terms of difference in position. If they veer
   * outside of this, an assertion will be triggered.
   */
  static constexpr const double kLANE_VECTOR_DIST_TOL = 0.2;

  /**
   * How close the robot needs to be to a given angle in order to be considered
   * to have that azimuth heading. If they veer outside of this range, an
   * assertion will be triggered while traversing the ingress/egress lanes.
   */
  static const rmath::radians kROBOT_AZIMUTH_TOL;

  /* inherited states */
  HFSM_ENTRY_INHERIT_ND(util_hfsm, entry_wait_for_signal);

  /* builder states */
  HFSM_STATE_DECLARE_ND(builder_fsm, start);
  HFSM_STATE_DECLARE(builder_fsm,
                     acquire_frontier_set,
                     csteer2D::ds::path_state);
  HFSM_ENTRY_DECLARE_ND(builder_fsm,
                        entry_acquire_frontier_set);
  HFSM_STATE_DECLARE(builder_fsm, wait_for_robot, const robot_wait_data);
  HFSM_STATE_DECLARE(builder_fsm,
                     acquire_placement_loc,
                     csteer2D::ds::path_state);
  HFSM_STATE_DECLARE(builder_fsm,
                     wait_for_block_place,
                     const rpfsm::event_data);
  HFSM_STATE_DECLARE(builder_fsm,
                     acquire_egress_lane,
                     csteer2D::ds::path_state);
  HFSM_STATE_DECLARE(builder_fsm,
                     structure_egress,
                     csteer2D::ds::path_state);
  HFSM_EXIT_DECLARE(builder_fsm, exit_structure_egress);

  HFSM_STATE_DECLARE_ND(builder_fsm, finished);

  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
    return &mc_state_map[index];
  }

  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ekST_MAX_STATES);

  /**
   * \brief Return \c TRUE if there is another robot too close to the current
   * robot's position, given the direction it is heading (i.e. if it is close to
   * another robot that is orthogonal to where it is traveling, that is
   * ignored), and \c FALSE otherwise.
   */

  bool robot_trajectory_proximity(void) const;

  /**
   * \brief Return \c TRUE if there is another robot too close to the current
   * robot's position, according to the L-shaped motion patterns of chess
   * knights.
   *
   * This is only needed when acquiring a block placement location in order to
   * avoid (more) special cases in the builder algorithm.
   */
  bool robot_manhattan_proximity(void) const;

  /**
   * \brief Verify that the robot position is still (mostly) parallel to the
   * vector defining the construction lane.
   */
  bool lane_alignment_verify_pos(const rmath::vector2d& lane_point,
                                 const rmath::radians& orientation) const;

  /**
   * \brief Verify that the headingposition is still (mostly) parallel to the
   * angle of the vector defining the construction lane.
   */
  bool lane_alignment_verify_azimuth(const rmath::radians& orientation) const;

  /**
   * \brief Retur the stygmergic configuration the robot has encountered when it
   * has reached the frontier set, and an empty configuration otherwise.
   */
  stygmergic_configuration frontier_set_acquisition(void) const;

  /**
   * \brief Calculate the path from the robot's current position to the frontier
   * set.
   */
  std::vector<rmath::vector2d> calc_frontier_set_path(void) const;

  /**
   * \brief Calculate the path from the robot's current position to the egress
   * lane (this path might be empty if the robot was already in the egress lane
   * when it placed its block).
   */
  std::vector<rmath::vector2d> calc_path_to_egress(void) const;

  /**
   * \brief Calculate the path from the robot's current position in the egress
   * lane off of the structure and back to the 2D arena.
   */
  std::vector<rmath::vector2d> calc_egress_path(void);

  /**
   * \brief Calculate the path from the location at which the robot has reached
   * the frontier set to an appropriate cell from which to place its block,
   * given the current shape of the frontier set it encounters.
   */
  std::vector<rmath::vector2d> calc_placement_path(
      stygmergic_configuration acq) const;

  /* clang-format off */
  const scperception::builder_perception_subsystem* mc_perception;

  repr::construction_lane                           m_lane{};
  /* clang-format on */
};

NS_END(fsm, silicon);

#endif /* INCLUDE_SILICON_FSM_BUILDER_FSM_HPP_ */
