/**
 * \file ingress_path.hpp
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
#include <vector>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace prism::repr {
class construction_lane;
} /* namespace prism::repr */

namespace cosm::subsystem {
class sensing_subsystemQ3D;
} /* namespace cosm::subsystem */

namespace prism::controller::perception {
class builder_perception_subsystem;
} /* namespace prism::controller::perception */

NS_START(prism, fsm, calculators);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ingress_path
 * \ingroup fsm calculators
 *
 * \brief Once a robot has reached the ingress point of its chosen construction
 * lane, calculate the path to the "frontier set", which is defined as the set
 * of cells adjacent to cells containing the latest set of placed blocks. The
 * exact location of this set is unknown to the robot a priori, as other robots
 * could have placed blocks within this last since it last placed one, so the
 * calculated path is to the BACK of the ingress lane; once the robot encounters
 * a stygmetric configuration that is a member of the frontier set, it will go
 * on to the next phase of the builder FSM.
 */
class ingress_path : public rer::client<ingress_path> {
 public:
  ingress_path(const csubsystem::sensing_subsystemQ3D* sensing,
               const pcperception::builder_perception_subsystem* perception);

  std::vector<rmath::vector2d> operator()(
      const prepr::construction_lane* lane) const;

  /* Not move/copy constructable/assignable by default */
  ingress_path(const ingress_path&) = delete;
  const ingress_path& operator=(const ingress_path&) = delete;
  ingress_path(ingress_path&&) = delete;
  ingress_path& operator=(ingress_path&&) = delete;

 private:
  /* clang-format off */
  const csubsystem::sensing_subsystemQ3D*           mc_sensing;
  const pcperception::builder_perception_subsystem* mc_perception;
  /* clang-format on */
};

NS_END(calculators, fsm, prism);

