/**
 * \file builder_util_fsm.hpp
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

#ifndef INCLUDE_SILICON_FSM_BUILDER_UTIL_FSM_HPP_
#define INCLUDE_SILICON_FSM_BUILDER_UTIL_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>
#include <vector>

#include "rcppsw/er/client.hpp"

#include "cosm/robots/footbot/footbot_subsystem_fwd.hpp"
#include "cosm/spatial/fsm/util_hfsm.hpp"
#include "cosm/ta/taskable.hpp"

#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace silicon::controller::perception {
class builder_perception_subsystem;
} // namespace silicon::controller::perception

namespace silicon::repr {
class construction_lane;
} /* namespace silicon::repr */

NS_START(silicon, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class builder_util_fsm
 * \ingroup fsm
 *
 * \brief The Base FSM for navigating a chosen construction lane once the robot
 * has entered the construction area to place the block it is carrying at an
 * appropriate place on the structure. Does not contain an FSM per-se, and is
 * more of a common set of states/routines needed by the various pieces of the
 * builder FSM.
 */
class builder_util_fsm : public csfsm::util_hfsm,
                         public cta::taskable,
                         public rer::client<builder_util_fsm> {
 public:
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

  builder_util_fsm(const scperception::builder_perception_subsystem* perception,
                   crfootbot::footbot_saa_subsystem* saa,
                   rmath::rng* rng,
                   uint8_t max_states);

  ~builder_util_fsm(void) override;

  /* not copy constructible or copy assignable by default */
  builder_util_fsm(const builder_util_fsm&) = delete;
  builder_util_fsm& operator=(const builder_util_fsm&) = delete;

  /* HFSM overrides */
  void init(void) override;

 protected:
  enum class robot_proximity_type {
    /**
     * Consider robot proximity via Euclidean distance, but ONLY along the
     * current robot's trajectory (i.e., a robot passing this one in the other
     * lane of the construction lane would not be considered within proximity,
     * regardless of distance).
     */
    ekTRAJECTORY,

    /**
     * Consider robot proximity via Manhattan distance, in which any robot a
     * distance of 3 units or less will be considered within proximity.
     */
    ekMANHATTAN
  };

  struct robot_wait_data final : public rpfsm::event_data {
    explicit robot_wait_data(robot_proximity_type in) : prox_type(in) {}
    robot_proximity_type prox_type;
  };

  const scperception::builder_perception_subsystem* perception(void) const {
    return mc_perception;
  }

  const repr::construction_lane* allocated_lane(void) const {
    return mc_alloc_lane;
  }
  void allocated_lane(const repr::construction_lane* l) { mc_alloc_lane = l; }

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

  /* builder states */
  HFSM_STATE_DECLARE(builder_util_fsm, wait_for_robot, const robot_wait_data);

 private:
  /* clang-format off */
  const scperception::builder_perception_subsystem* mc_perception;
  const srepr::construction_lane*                   mc_alloc_lane{nullptr};
  /* clang-format on */
};

NS_END(fsm, silicon);

#endif /* INCLUDE_SILICON_FSM_BUILDER_UTIL_FSM_HPP_ */
