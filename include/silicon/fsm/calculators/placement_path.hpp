/**
 * \file placement_path.hpp
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

#ifndef INCLUDE_SILICON_FSM_CALCULATORS_PLACEMENT_PATH_HPP_
#define INCLUDE_SILICON_FSM_CALCULATORS_PLACEMENT_PATH_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

#include "silicon/repr/fs_configuration.hpp"
#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace silicon::repr {
class construction_lane;
} /* namespace silicon::repr */

namespace silicon::controller::perception {
class builder_perception_subsystem;
} /* namespace silicon::controller::perception */

namespace cosm::subsystem {
class sensing_subsystemQ3D;
} /* namespace cosm::subsystem */

NS_START(silicon, fsm, calculators);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class placement_path
 * \ingroup fsm calculators
 *
 * \brief Once a robot has reach the frontier set and acquired a specific
 * stygmetrig configuration (acquired meaning it has entered the robot's LOS),
 * calculate a path to a cell adjacent to the cell the robot will place a block
 * in (the placement site), so that block placement will be triggered by the
 * loop functions.
 */
class placement_path : public rer::client<placement_path> {
 public:
  placement_path(
      const csubsystem::sensing_subsystemQ3D* sensing,
      const scperception::builder_perception_subsystem* perception);

  std::vector<rmath::vector2d> operator()(
      const srepr::construction_lane* lane,
      const srepr::fs_configuration& acq) const;

  /* Not move/copy constructable/assignable by default */
  placement_path(const placement_path&) = delete;
  const placement_path& operator=(const placement_path&) =
      delete;
  placement_path(placement_path&&) = delete;
  placement_path& operator=(placement_path&&) = delete;

 private:
  /* clang-format off */
  const csubsystem::sensing_subsystemQ3D*           mc_sensing;
  const scperception::builder_perception_subsystem* mc_perception;
  /* clang-format on */
};

NS_END(calculators, fsm, silicon);

#endif /* INCLUDE_SILICON_FSM_CALCULATORS_PLACEMENT_PATH_HPP_ */
