/**
 * \file block_vanished.hpp
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

#ifndef INCLUDE_SILICON_CONTROLLER_OPERATIONS_BLOCK_VANISHED_HPP_
#define INCLUDE_SILICON_CONTROLLER_OPERATIONS_BLOCK_VANISHED_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "silicon/controller/controller_fwd.hpp"
#include "silicon/fsm/fsm_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, controller, operations, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_vanished
 * \ingroup operations detail
 *
 * \brief Created whenever a robot is serving a block pickup penalty, but while
 * serving the penalty the block it is waiting for vanishes due to another
 * robot picking it up.
 */
class block_vanished : public rer::client<block_vanished> {
 private:
  struct visit_typelist_impl {
    using controllers = controller::typelist;
    using fsms = rmpl::typelist<fsm::fcrw_bst_fsm>;

    using value = boost::mpl::joint_view<controllers::type,
                                         fsms::type>::type;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  explicit block_vanished(const rtypes::type_uuid& block_id);
  ~block_vanished(void) override = default;

  block_vanished(const block_vanished&) = delete;
  block_vanished& operator=(const block_vanished&) = delete;

  /* single target construction */
  void visit(controller::fcrw_bst_controller& controller);
  void visit(fsm::fcrw_bst_fsm& fsm);

 private:
  /* clang-format off */
  const rtypes::type_uuid mc_block_id;
  /* clang-format on */
};

NS_END(detail);

using block_vanished_visitor = rpvisitor::filtered_visitor<detail::block_vanished>;

NS_END(operations, controller, silicon);

#endif /* INCLUDE_SILICON_CONTROLLER_OPERATIONS_BLOCK_VANISHED_HPP_ */
