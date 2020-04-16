/**
 * \file cell3D_block_extent.cpp
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
#include "silicon/structure/operations/cell3D_block_extent.hpp"

#include "cosm/ds/cell3D.hpp"

#include "silicon/structure/structure3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, structure, operations, detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cell3D_block_extent::cell3D_block_extent(const rmath::vector3z& coord,
                                         crepr::base_block3D* block)
    : ER_CLIENT_INIT("silicon.structure.operations.cell3D_block_extent"),
      cell3D_op(coord),
      m_block(block) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cell3D_block_extent::visit(cds::cell3D& cell) {
  cell.entity(m_block);
  visit(cell.fsm());
} /* visit() */

void cell3D_block_extent::visit(cfsm::cell3D_fsm& fsm) {
  ER_ASSERT(!fsm.state_has_block() && !fsm.state_in_block_extent(),
            "FSM in wrong state");
  fsm.event_block_extent();
} /* visit() */

NS_END(detail, operations, structure, silicon);
