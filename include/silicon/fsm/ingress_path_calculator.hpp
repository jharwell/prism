/**
 * \file ingress_path_calculator.hpp
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

#ifndef INCLUDE_SILICON_FSM_INGRESS_PATH_CALCULATOR_HPP_
#define INCLUDE_SILICON_FSM_INGRESS_PATH_CALCULATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace silicon::repr {
class construction_lane;
} /* namespace silicon::repr */

namespace cosm::subsystem {
class sensing_subsystemQ3D;
} /* namespace cosm::subsystem */

NS_START(silicon, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ingress_path_calculator
 * \ingroup fsm
 *
 * \brief Once a robot is sufficiently close to the X/Y coordinate of the ingress
 * point for a construction lane (which dimension depends on orientation),
 * calculate the path TO the ingress point.
 */
class ingress_path_calculator : public rer::client<ingress_path_calculator> {
 public:
  explicit ingress_path_calculator(
      const csubsystem::sensing_subsystemQ3D* sensing);

  std::vector<rmath::vector2d> operator()(
      const srepr::construction_lane* lane) const;

  /* Not move/copy constructable/assignable by default */
  ingress_path_calculator(const ingress_path_calculator&) = delete;
  const ingress_path_calculator& operator=(const ingress_path_calculator&) =
      delete;
  ingress_path_calculator(ingress_path_calculator&&) = delete;
  ingress_path_calculator& operator=(ingress_path_calculator&&) = delete;

 private:
  /* clang-format off */
  const csubsystem::sensing_subsystemQ3D* mc_sensing;
  /* clang-format on */
};

NS_END(fsm, silicon);

#endif /* INCLUDE_SILICON_FSM_INGRESS_PATH_CALCULATOR_HPP_ */
