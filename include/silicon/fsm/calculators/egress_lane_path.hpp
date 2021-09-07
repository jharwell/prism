/**
 * \file egress_lane_path.hpp
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

#ifndef INCLUDE_SILICON_FSM_CALCULATORS_EGRESS_LANE_PATH_HPP_
#define INCLUDE_SILICON_FSM_CALCULATORS_EGRESS_LANE_PATH_HPP_

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

NS_START(silicon, fsm, calculators);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class egress_lane_path
 * \ingroup fsm calculators
 *
 * \brief Once a robot has placed its block on its chosen construction target,
 * calculate the path TO the egress lane within its allocated construction lane.
 */
class egress_lane_path : public rer::client<egress_lane_path> {
 public:
  explicit egress_lane_path(
      const csubsystem::sensing_subsystemQ3D* sensing);

  std::vector<rmath::vector2d> operator()(
      const srepr::construction_lane* lane) const;

  /* Not move/copy constructable/assignable by default */
  egress_lane_path(const egress_lane_path&) = delete;
  const egress_lane_path& operator=(const egress_lane_path&) =
      delete;
  egress_lane_path(egress_lane_path&&) = delete;
  egress_lane_path& operator=(egress_lane_path&&) = delete;

 private:
  /* clang-format off */
  const csubsystem::sensing_subsystemQ3D* mc_sensing;
  /* clang-format on */
};

NS_END(calculators, fsm, silicon);

#endif /* INCLUDE_SILICON_FSM_CALCULATORS_EGRESS_LANE_PATH_HPP_ */
