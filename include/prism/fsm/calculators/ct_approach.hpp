/**
 * \file ct_approach.hpp
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

#ifndef INCLUDE_PRISM_FSM_CALCULATORS_CT_APPROACH_HPP_
#define INCLUDE_PRISM_FSM_CALCULATORS_CT_APPROACH_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/radians.hpp"
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
 * \class ct_approach
 * \ingroup fsm calculators
 *
 * \brief Calculates (1) if the robot is on the correct face of the target structure
 * when it picks up a block (2) if it is close enough to its chosen construction
 * lane to begin the entry sequence (3) the orthogonal distance to the ingress
 * point on its chosen construction lane if (2) is violated.
 *
 * Calculation results used to drive the robot to the necessary face using 2D
 * steering forces.
 */
class ct_approach : public rer::client<ct_approach> {
 public:
  struct ct_approach_vector {
    bool x_ok{false};
    bool y_ok{false};
    rtypes::spatial_dist orthogonal_dist{0.0};
    rmath::radians ingress_angle{rmath::radians::kZERO};
  };
  ct_approach(const csubsystem::sensing_subsystemQ3D* sensing,
              const rtypes::spatial_dist& lane_alignment_tol);

  ct_approach_vector operator()(const prepr::construction_lane* lane) const;

  /* Not move/copy constructable/assignable by default */
  ct_approach(const ct_approach&) = delete;
  const ct_approach& operator=(const ct_approach&) = delete;
  ct_approach(ct_approach&&) = delete;
  ct_approach& operator=(ct_approach&&) = delete;

 private:
  /* clang-format off */
  const rtypes::spatial_dist              mc_lane_alignment_tol;
  const csubsystem::sensing_subsystemQ3D* mc_sensing;
  /* clang-format on */
};

NS_END(calculators, fsm, prism);

#endif /* INCLUDE_PRISM_FSM_CALCULATORS_CT_APPROACH_HPP_ */
