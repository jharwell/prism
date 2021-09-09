/**
 * \file builder_prox_type.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_PRISM_CONTROLLER_PERCEPTION_BUILDER_PROX_TYPE_HPP_
#define INCLUDE_PRISM_CONTROLLER_PERCEPTION_BUILDER_PROX_TYPE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, controller, perception);

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * \brief The type of proximity to check and/or the type of proximity
 * detected. Different types of proximities are relevant at different stages of
 * the \ref pfsm::builder_fsm.
 *
 * \ingroup controller perception
 */
enum class builder_prox_type {
  /**
   * No robots nearby that we need to worry about while on the structure.
   */
  ekNONE = 0,

  /**
   * Consider robot proximity via Euclidean distance, but ONLY along the
   * current robot's trajectory (i.e., a robot passing this one in the other
   * half of the construction lane would not be considered within proximity,
   * regardless of distance).
   */
  ekTRAJECTORY = 1 << 0,

  /**
   * Consider builder robots in front of the current one which are in the
   * frontier set; both the current and other robots must be in the frontier
   * set for this type of proximity to trigger.
   */
  ekFRONTIER_SET = 1 << 1
};

NS_END(perception, controller, prism);

#endif /* INCLUDE_PRISM_CONTROLLER_PERCEPTION_BUILDER_PROX_TYPE_HPP_ */
