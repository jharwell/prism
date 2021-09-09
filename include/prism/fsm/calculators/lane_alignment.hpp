/**
 * \file lane_alignment.hpp
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

#ifndef INCLUDE_PRISM_FSM_CALCULATORS_LANE_ALIGNMENT_HPP_
#define INCLUDE_PRISM_FSM_CALCULATORS_LANE_ALIGNMENT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/types/spatial_dist.hpp"

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

NS_START(prism, fsm, calculators);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class lane_alignment
 * \ingroup fsm calculators
 *
 * \brief Calculate the alignment of a robot to the ingress/egress lanes within
 * a construction lane in terms of position and orientation.
 */
class lane_alignment : public rer::client<lane_alignment> {
 public:
  struct ret_type {
    bool ingress{false};
    bool egress{false};
    bool azimuth{false};
  };

  /**
   * How close the robot needs to be to the vectors representing the
   * ingress/egress lanes in terms of difference in position.
   */
  static const rtypes::spatial_dist kTRAJECTORY_ORTHOGONAL_TOL;

  /**
   * How close the robot needs to be to a the vectors representing the
   * ingress/egress lanes in terms of azimuth angle deviation.
   */
  static const rmath::radians kAZIMUTH_TOL;

  explicit lane_alignment(const csubsystem::sensing_subsystemQ3D* sensing)
      : ER_CLIENT_INIT("prism.fsm.calculator.lane_alignment"),
        mc_sensing(sensing) {}

  ret_type operator()(const prepr::construction_lane* lane) const;

  /* Not move/copy constructable/assignable by default */
  lane_alignment(const lane_alignment&) = delete;
  const lane_alignment& operator=(const lane_alignment&) = delete;
  lane_alignment(lane_alignment&&) = delete;
  lane_alignment& operator=(lane_alignment&&) = delete;

 private:
  bool verify_pos(const rmath::vector2d& lane_point,
                  const rmath::radians& orientation) const;
  bool verify_azimuth(const rmath::radians& orientation) const;

  /* clang-format off */
  const csubsystem::sensing_subsystemQ3D* mc_sensing;
  /* clang-format on */
};

NS_END(calculators, fsm, prism);

#endif /* INCLUDE_PRISM_FSM_CALCULATORS_LANE_ALIGNMENT_HPP_ */
