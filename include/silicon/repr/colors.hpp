/**
 * \file colors.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_SILICON_REPR_COLORS_HPP_
#define INCLUDE_SILICON_REPR_COLORS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/utils/color.hpp"

#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
struct colors {
  /**
   * \brief A robot running the \ref sfsm::acquire_block_placement_site_fsm or
   * \ref sfsm::structure_egress_fsm.
   */
  static const rutils::color& builder(void) {
    static rutils::color kBuilder = rutils::color::kBLUE;
    return kBuilder;
  }

  /**
   * \brief A robot waiting for a signal during FSM operation.
   */
  static const rutils::color& wait_signal(void) {
    static rutils::color kWaitSignal = rutils::color::kWHITE;
    return kWaitSignal;
  }

  /**
   * \brief A robot approaching a construction target via the \ref
   * sfsm::structure_ingress_fsm.
   */
  static const rutils::color& ct_approach(void) {
    static rutils::color kCTApproach = rutils::color::kGREEN;
    return kCTApproach;
  }

  /**
   * \brief The colors used by robots to identify to others what they are
   * currently doing while engaged in construction related tasks.
   */
  static std::array<rutils::color, 3>& ct(void) {
    static std::array<rutils::color, 3> ct = { builder(),
                                               wait_signal(),
                                               ct_approach() };
    return ct;
  }
};

NS_END(repr, silicon);

#endif /* INCLUDE_SILICON_REPR_COLORS_HPP_ */
