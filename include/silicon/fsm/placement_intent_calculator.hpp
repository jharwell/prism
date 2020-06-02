/**
 * \file placement_intent_calculator.hpp
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

#ifndef INCLUDE_SILICON_FSM_PLACEMENT_INTENT_CALCULATOR_HPP_
#define INCLUDE_SILICON_FSM_PLACEMENT_INTENT_CALCULATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

#include "silicon/fsm/block_placer.hpp"
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

NS_START(silicon, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class placement_intent_calculator
 * \ingroup fsm
 *
 * \brief Once a robot has reached the end of its placement end, it will be
 * adjacent to a cell it wants to place a block into; this class calculates that
 * cell, which can vary depending on structure orientation and current robot
 * position on the structure, so the calculation is pulled out into its own
 * class.
 */
class placement_intent_calculator
    : public rer::client<placement_intent_calculator> {
 public:
  placement_intent_calculator(
      const csubsystem::sensing_subsystemQ3D* sensing,
      const scperception::builder_perception_subsystem* perception);

  block_placer::placement_intent operator()(
      const srepr::construction_lane* lane) const;

  /* Not move/copy constructable/assignable by default */
  placement_intent_calculator(const placement_intent_calculator&) = delete;
  const placement_intent_calculator& operator=(
      const placement_intent_calculator&) = delete;
  placement_intent_calculator(placement_intent_calculator&&) = delete;
  placement_intent_calculator& operator=(placement_intent_calculator&&) = delete;

 private:
  /* clang-format off */
  const csubsystem::sensing_subsystemQ3D*           mc_sensing;
  const scperception::builder_perception_subsystem* mc_perception;
  /* clang-format on */
};

NS_END(fsm, silicon);

#endif /* INCLUDE_SILICON_FSM_PLACEMENT_INTENT_CALCULATOR_HPP_ */
