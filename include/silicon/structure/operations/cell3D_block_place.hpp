/**
 * \file cell3D_block_place.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_OPERATIONS_CELL3D_BLOCK_PLACE_HPP_
#define INCLUDE_SILICON_STRUCTURE_OPERATIONS_CELL3D_BLOCK_PLACE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/ds/operations/cell3D_op.hpp"
#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace silicon::structure {
class structure3D;
} /* namespace silicon::structure */

NS_START(silicon, structure, operations, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cell3D_block_place
 * \ingroup structure operations
 *
 * \brief Change the state of a  \ref cell3D from some state to hosting a block
 * of some kind.
 */
class cell3D_block_place : public rer::client<cell3D_block_place>,
                           public cdops::cell3D_op {
 private:
  struct visit_typelist_impl {
    using inherited = cell3D_op::visit_typelist;
    using value = inherited;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  cell3D_block_place(const rmath::vector3u& loc, crepr::base_block3D* block);
  cell3D_block_place& operator=(const cell3D_block_place&) = delete;
  cell3D_block_place(const cell3D_block_place&) = delete;

  void visit(cds::cell3D& cell);

 private:
  void visit(cfsm::cell3D_fsm& fsm);

  /* clang-format off */
  crepr::base_block3D* m_block;
  /* clang-format on */
};

/**
 * \brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cell3D_block_place_visitor_impl =
    rpvisitor::precise_visitor<detail::cell3D_block_place,
                               detail::cell3D_block_place::visit_typelist>;

NS_END(detail);

class cell3D_block_place_visitor : public detail::cell3D_block_place_visitor_impl {
  using detail::cell3D_block_place_visitor_impl::cell3D_block_place_visitor_impl;
};

NS_END(events, foraging, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_OPERATIONS_CELL3D_BLOCK_PLACE_HPP_ */
