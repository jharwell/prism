/**
 * \file ct_approach_calculator.hpp
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

#ifndef INCLUDE_SILICON_FSM_CT_APPROACH_CALCULATOR_HPP_
#define INCLUDE_SILICON_FSM_CT_APPROACH_CALCULATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"

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
 * \class ct_approach_calculator
 * \ingroup fsm
 *
 * \brief Calculates (1) if the robot is on the correct face of the target structure
 * when it picks up a block (2) if it is close enough to its chosen construction
 * lane to begin the entry sequence (3) the orthogonal distance to the ingress
 * point on its chosen construction lane if (2) is violated.
 *
 * Calculation results used to drive the robot to the necessary face using 2D
 * steering forces.
 */
class ct_approach_calculator : public rer::client<ct_approach_calculator> {
 public:
  struct ct_approach_ret {
    bool x_ok{false};
    bool y_ok{false};
    double orthogonal_dist{-1};
  };
  ct_approach_calculator(const csubsystem::sensing_subsystemQ3D* sensing,
                         double lane_alignment_tol);

  ct_approach_ret operator()(const srepr::construction_lane* lane) const;

  /* Not move/copy constructable/assignable by default */
  ct_approach_calculator(const ct_approach_calculator&) = delete;
  const ct_approach_calculator& operator=(const ct_approach_calculator&) = delete;
  ct_approach_calculator(ct_approach_calculator&&) = delete;
  ct_approach_calculator& operator=(ct_approach_calculator&&) = delete;

 private:
  /* clang-format off */
  const double                            mc_lane_alignment_tol;
  const csubsystem::sensing_subsystemQ3D* mc_sensing;
  /* clang-format on */
};

NS_END(fsm, silicon);

#endif /* INCLUDE_SILICON_FSM_CT_APPROACH_CALCULATOR_HPP_ */
