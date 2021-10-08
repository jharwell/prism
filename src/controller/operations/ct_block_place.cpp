/**
 * \file ct_block_place.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/controller/operations/ct_block_place.hpp"

#include "cosm/repr/base_block3D.hpp"

#include "prism/controller/fcrw_bst_controller.hpp"
#include "prism/fsm/construction_signal.hpp"
#include "prism/fsm/fcrw_bst_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, controller, operations, detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
ct_block_place::ct_block_place(const rtypes::type_uuid& robot_id)
    : mc_robot_id(robot_id) {}

/*******************************************************************************
 * Single Target Construction
 ******************************************************************************/
void ct_block_place::visit(controller::fcrw_bst_controller& controller) {
  controller.ndc_uuid_push();

  visit(*controller.fsm());

  controller.ndc_uuid_pop();
} /* visit() */

void ct_block_place::visit(fsm::fcrw_bst_fsm& fsm) {
  fsm.inject_event(fsm::construction_signal::ekCT_BLOCK_PLACE,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

NS_END(detail, operations, controller, prism);
