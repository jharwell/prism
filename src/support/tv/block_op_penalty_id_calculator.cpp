/**
 * \file block_op_penalty_id_calculator.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/support/tv/block_op_penalty_id_calculator.hpp"

#include "cosm/repr/base_block3D.hpp"

#include "silicon/controller/constructing_controller.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, support, tv);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_op_penalty_id_calculator::block_op_penalty_id_calculator(
    const carena::base_arena_map* map)
    : ER_CLIENT_INIT("silicon.support.tv.block_op_penalty_id_calculator"),
      mc_map(map) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rtypes::type_uuid block_op_penalty_id_calculator::operator()(
    const controller::constructing_controller& controller,
    const block_op_src& src) const {
  rtypes::type_uuid id = rtypes::constants::kNoUUID;
  switch (src) {
    case block_op_src::ekARENA_PICKUP:
      id = decoratee().from_free_pickup(
          controller.rpos2D(), controller.entity_acquired_id(), mc_map);
      break;
    case block_op_src::ekCT_BLOCK_MANIP:
      ER_ASSERT(nullptr != controller.block() &&
                    rtypes::constants::kNoUUID != controller.block()->id(),
                "Robot not carrying block?");
      id = controller.block()->id();
      break;
    default:
      break;
  }
  return id;
} /* penalty_id_calc() */

NS_END(tv, support, silicon);
