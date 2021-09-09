/**
 * \file block_placer.hpp
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

#ifndef INCLUDE_PRISM_FSM_BLOCK_PLACER_HPP_
#define INCLUDE_PRISM_FSM_BLOCK_PLACER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>

#include "rcppsw/math/vector3.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "prism/repr/placement_intent.hpp"
#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_placer
 * \ingroup fsm
 *
 * \brief Interface defining functions which controllers/FSMs that are top level
 * and need to interact with the arena need to implement in order for the robot
 * to be able to place blocks onto a target structure.
 */
class block_placer {
 public:
  block_placer(void) = default;
  virtual ~block_placer(void) = default;

  /**
   * \brief Return the intended block placement site, if there is not, and
   * nothing if the robot is not currently intending to place a block (e.g. it
   * has not yet reached its desired site).
   */
  virtual boost::optional<repr::placement_intent>
  block_placement_intent(void) const = 0;
};

NS_END(fsm, prism);

#endif /* INCLUDE_PRISM_FSM_BLOCK_PLACER_HPP_ */
