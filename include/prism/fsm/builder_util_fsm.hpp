/**
 * \file builder_util_fsm.hpp
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
#include <utility>
#include <vector>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/radians.hpp"

#include "cosm/spatial/fsm/util_hfsm.hpp"
#include "cosm/subsystem/subsystem_fwd.hpp"
#include "cosm/ta/taskable.hpp"

#include "prism/controller/perception/builder_prox_type.hpp"
#include "prism/fsm/calculators/lane_alignment.hpp"
#include "prism/repr/fs_configuration.hpp"
#include "prism/prism.hpp"
#include "prism/fsm/fsm_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace prism::controller::perception {
class builder_perception_subsystem;
} // namespace prism::controller::perception

namespace prism::repr {
class construction_lane;
} /* namespace prism::repr */

NS_START(prism, fsm);

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
   * \brief Verify that the robot position is still (mostly) parallel to the
   * vector defining the construction lane.
   */
  static bool
  lane_alignment_verify_pos(const csubsystem::sensing_subsystemQ3D* sensing,
                            const rmath::vector2d& lane_point,
                            const rmath::radians& orientation);

  /**
   * \brief Verify that the headingposition is still (mostly) parallel to the
   * angle of the vector defining the construction lane.
   */
  static bool
  lane_alignment_verify_azimuth(const csubsystem::sensing_subsystemQ3D* sensing,
                                const rmath::radians& orientation);

  builder_util_fsm(const pfsm::fsm_params* params,
                   rmath::rng* rng,
                   uint8_t max_states);

  ~builder_util_fsm(void) override;

  /* not copy constructible or copy assignable by default */
  builder_util_fsm(const builder_util_fsm&) = delete;
  builder_util_fsm& operator=(const builder_util_fsm&) = delete;

  /* HFSM overrides */
  void init(void) override;

 protected:
  struct robot_wait_data final : public rpfsm::event_data {
    explicit robot_wait_data(
        const pcperception::builder_prox_type& prox_in,
        prepr::fs_configuration fs_in = prepr::fs_configuration::ekNONE)
        : prox_type(prox_in), fs(fs_in) {}

    pcperception::builder_prox_type prox_type;
    prepr::fs_configuration fs;
  };

  const pcperception::builder_perception_subsystem* perception(void) const {
    return mc_perception;
  }

  const repr::construction_lane* allocated_lane(void) const {
    return mc_alloc_lane;
  }
  void allocated_lane(const repr::construction_lane* l) { mc_alloc_lane = l; }

  /* builder states */
  RCPPSW_HFSM_STATE_DECLARE(builder_util_fsm, wait_for_robot, robot_wait_data);
  RCPPSW_HFSM_ENTRY_DECLARE_ND(builder_util_fsm, entry_wait_for_robot);
  RCPPSW_HFSM_EXIT_DECLARE(builder_util_fsm, exit_wait_for_robot);

 private:
  /* clang-format off */
  const pcperception::builder_perception_subsystem* mc_perception;
  const prepr::construction_lane*                   mc_alloc_lane{nullptr};
  /* clang-format on */
};

NS_END(fsm, prism);

