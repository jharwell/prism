/**
 * \file structure_block_placement.hpp
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

#ifndef INCLUDE_SILICON_EVENTS_STRUCTURE_BLOCK_PLACEMENT_HPP_
#define INCLUDE_SILICON_EVENTS_STRUCTURE_BLOCK_PLACEMENT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"

#include "cosm/events/cell3D_op.hpp"
#include "cosm/repr/base_block3D.hpp"

#include "silicon/silicon.hpp"
#include "silicon/structure/placement_orientation.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace silicon::structure {
class structure3D;
} /* namespace silicon::structure */

NS_START(silicon, events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class structure_block_placement
 * \ingroup events
 *
 * \brief A block drop to place a block onto the in-progress \ref structure3D.
 */
class structure_block_placement : public rer::client<structure_block_placement>,
                                  public cevents::cell3D_op {
 public:
 private:
  struct visit_typelist_impl {
    using inherited = cell3D_op::visit_typelist;
    using others = rmpl::typelist<structure::structure3D>;
    using value = boost::mpl::joint_view<inherited::type, others::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  structure_block_placement(const rmath::vector3u& loc,
                  const structure::placement_orientation& orientation,
                  crepr::base_block3D* block);

  /* Not copy constructable/assignable by default */
  structure_block_placement(const structure_block_placement&) = delete;
  const structure_block_placement& operator=(const structure_block_placement&) = delete;

  void visit(structure::structure3D& structure);

 private:
  void visit(cds::cell3D& cell);
  void visit(cfsm::cell3D_fsm& fsm);

  /* clang-format off */
  const structure::placement_orientation mc_orientation;

  crepr::base_block3D*                   m_block;
  /* clang-format on */
};

/**
 * \brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using structure_block_placement_visitor_impl =
    rpvisitor::precise_visitor<structure_block_placement,
                               structure_block_placement::visit_typelist>;

NS_END(detail);

class structure_block_placement_visitor : public detail::structure_block_placement_visitor_impl {
  using detail::structure_block_placement_visitor_impl::structure_block_placement_visitor_impl;
};


NS_END(events, silicon);

#endif /* INCLUDE_SILICON_EVENTS_STRUCTURE_BLOCK_PLACEMENT_HPP_ */
