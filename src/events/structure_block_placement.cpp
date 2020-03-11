/**
 * \file structure_block_placement.cpp
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
#include "silicon/events/structure_block_placement.hpp"

#include "cosm/ds/cell3D.hpp"

#include "silicon/structure/structure3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, events, detail);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
structure_block_placement::structure_block_placement(const rmath::vector3u& loc,
                                 const structure::placement_orientation& orientation,
                                 crepr::base_block3D* block)
    : ER_CLIENT_INIT("silicon.events.structure_block_placement"),
      cell3D_op(loc),
      mc_orientation(orientation),
      m_block(block) {
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void structure_block_placement::visit(structure::structure3D& structure) {
  ER_ASSERT(structure.block_addition_valid(coord(), mc_orientation, m_block),
            "Bad placement of block%d @loc%s",
            m_block->id().v(),
            coord().to_str().c_str());
  structure.block_add(m_block);
  visit(structure[coord()]);
} /* visit() */

void structure_block_placement::visit(cds::cell3D& cell) {
  cell.entity(m_block);
  visit(cell.fsm());
} /* visit() */

void structure_block_placement::visit(cfsm::cell3D_fsm& fsm) {
  ER_ASSERT(!fsm.state_has_block() && !fsm.state_in_block_extent(),
            "FSM in wrong state");
  fsm.event_block_place();
} /* visit() */

NS_END(detail, events, silicon);
