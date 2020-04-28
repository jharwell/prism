/**
 * \file building_signal.hpp
 *
 * \copyright 202 John Harwell, All rights reserved.
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

#ifndef INCLUDE_SILICON_FSM_BUILDING_SIGNAL_HPP_
#define INCLUDE_SILICON_FSM_BUILDING_SIGNAL_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/fsm/util_signal.hpp"

#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class building_signal
 * \ingroup fsm
 *
 * \brief Signals that FSMs can use to communicate between sub/super states, and
 * that can be used to direct them in some way.
 */
class building_signal : public cfsm::util_signal {
 public:
  enum type {
    /**
     * \brief A robot has placed a block on the structure
     */
    ekBLOCK_PLACE = cfsm::util_signal::type::ekEXTERNAL_SIGNALS,
  };
};

NS_END(controller, silicon);

#endif /* INCLUDE_SILICON_FSM_BUILDING_SIGNAL_HPP_ */
