/**
 * \file builder_prox_checker.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_SILICON_CONTROLLER_PERCEPTION_BUILDER_PROX_CHECKER_HPP_
#define INCLUDE_SILICON_CONTROLLER_PERCEPTION_BUILDER_PROX_CHECKER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/utils/color.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/subsystem/subsystem_fwd.hpp"

#include "silicon/silicon.hpp"

#include "silicon/controller/perception/builder_prox_type.hpp"
#include "silicon/repr/fs_configuration.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace silicon::repr {
class construction_lane;
} /* namespace silicon:: */

NS_START(silicon, controller, perception);

class builder_perception_subsystem;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class builder_prox_checker
 * \ingroup controller perception
 *
 * \brief For a robot executing the \ref fsm::builder_fsm, check for the
 * proximity of other builder robots.
 */
class builder_prox_checker : public rer::client<builder_prox_checker> {

public:
  builder_prox_checker(const scperception::builder_perception_subsystem* perception,
                       const csubsystem::sensing_subsystemQ3D* sensing)
      : ER_CLIENT_INIT("silicon.controller.perception.builder_prox_checker"),
      mc_perception(perception),
        mc_sensing(sensing) {}

  /* Not move/copy constructable/assignable by default */
  builder_prox_checker(const builder_prox_checker&) = delete;
  builder_prox_checker& operator=(const builder_prox_checker&) = delete;
  builder_prox_checker(builder_prox_checker&&) = delete;
  builder_prox_checker& operator=(builder_prox_checker&&) = delete;

  builder_prox_type operator()(const srepr::construction_lane* lane,
                               const srepr::fs_configuration& fs) const;

  double trajectory_prox_dist(void) const;
  double frontier_set_prox_dist(void) const;

 private:
  struct lane_share_state {
    bool is_shared{false};
    bool other_in_ingress{false};
    bool other_in_egress{false};
    bool self_in_ingress{false};
    bool self_in_egress{false};
  };

  struct robot_compass_orientation {
    bool pos_x;
    bool neg_x;
    bool pos_y;
    bool neg_y;
  };

  static constexpr size_t kFRONTIER_SET_PROX_CELLS = 3;
  static constexpr size_t kTRAJECTORY_PROX_CELLS = 3;

  /**
   * \brief Return \c TRUE if there is another robot close to the current
   * robot's position such that we need to wait for it to move to continue
   * moving.
   */
  builder_prox_type robot_proximity(srepr::fs_configuration fs =
                                    srepr::fs_configuration::ekNONE) const;

  /**
   * \brief Return \ref builder_prox_type::ekTRAJECTORY if there is another
   * robot too close to the current robot's position, given the direction it is
   * heading (i.e. if it is close to another robot that is orthogonal to where
   * it is traveling, that is ignored), and \ref builder_prox_type::ekNONE
   * otherwise. Uses Euclidean distance measure.
   */

  builder_prox_type trajectory_proximity(
      const srepr::construction_lane* lane) const;

  /**
   * \brief Return \ref builder_prox_type::ekFRONTIER_SET if there is another
   * robot too close to the current robot's position if the current robot has
   * acquired the frontier set within the \ref construction_lane, and \ref
   * builder_prox_type::ekNONE otherwise.
   */
  builder_prox_type frontier_set_proximity(
      const srepr::fs_configuration& fs,
      const srepr::construction_lane* lane) const;

  /**
   * \brief Calculate whether or not the current robot is sharing a construction
   * lane with another robot with ABSOLUTE position \p other_rpos.
   */
  lane_share_state share_state_calc(const rmath::vector2d& other_rpos,
                                    const srepr::construction_lane* lane) const;

  /**
   * \brief Calculate the compass orientation (-X,+X,-Y,+Y) for the current
   * robot.
   */
  robot_compass_orientation self_orientation(void) const;

  /**
   * \brief Calculate whether or not the other robot with color \p color is a
   * builder robot we should be concerned about, or if it can safely be
   * ignored.
   */
  bool is_builder_robot(const rutils::color& color) const;

  /* clang-format off */
  const scperception::builder_perception_subsystem* mc_perception;
  const csubsystem::sensing_subsystemQ3D*           mc_sensing;
  /* clang-format on */
};

NS_END(perception, controller, silicon);

#endif /* INCLUDE_SILICON_CONTROLLER_PERCEPTION_BUILDER_PROX_CHECKER_HPP_ */
