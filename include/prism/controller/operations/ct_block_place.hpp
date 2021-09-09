/**
 * \file ct_block_place.hpp
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

#ifndef INCLUDE_PRISM_CONTROLLER_OPERATIONS_CT_BLOCK_PLACE_HPP_
#define INCLUDE_PRISM_CONTROLLER_OPERATIONS_CT_BLOCK_PLACE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/visitor/visitor.hpp"
#include "rcppsw/types/type_uuid.hpp"
#include "rcppsw/math/vector3.hpp"

#include "prism/controller/controller_fwd.hpp"
#include "prism/fsm/fsm_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, controller, operations, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ct_block_place
 * \ingroup operations detail
 *
 * \brief Fired whenever a block is placed on a construction target.
 */
class ct_block_place {
 private:
  struct visit_typelist_impl {
    using controllers = controller::typelist;
    using fsms = rmpl::typelist<fsm::fcrw_bst_fsm>;
    using value = boost::mpl::joint_view<controllers::type, fsms::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  ~ct_block_place(void) = default;

  ct_block_place(const ct_block_place&) = delete;
  ct_block_place& operator=(const ct_block_place&) = delete;

  /* Single target construction */
  void visit(controller::fcrw_bst_controller& controller);
  void visit(fsm::fcrw_bst_fsm& fsm);

 protected:
  explicit ct_block_place(const rtypes::type_uuid& robot_id);

 private:
  /* clang-format on */
  const rtypes::type_uuid mc_robot_id;
  /* clang-format off */
};

NS_END(detail);

using ct_block_place_visitor = rpvisitor::filtered_visitor<detail::ct_block_place>;

NS_END(operations, controller, prism);

#endif /* INCLUDE_PRISM_CONTROLLER_OPERATIONS_CT_BLOCK_PLACE_HPP_ */
