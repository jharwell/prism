/**
 * \file diagnostics.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/utils/color.hpp"

#include "cosm/hal/actuators/diagnostic_actuator.hpp"

#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, repr, diagnostics);

/*******************************************************************************
 * Types
 ******************************************************************************/
enum {
  /**
   * \brief A robot running the \ref pfsm::acquire_block_placement_site_fsm or
   * \ref pfsm::gmt_egress_fsm.
   */
  ekBUILDER = chactuators::diagnostics::ekMAX,

  /**
   * \brief A robot approaching a construction target via the \ref
   * pfsm::gmt_ingress_fsm.
   */
  ekCT_APPROACH,
};

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
extern chactuators::diagnostic_actuator::map_type kColorMap;

NS_END(diagnostics, repr, prism);
