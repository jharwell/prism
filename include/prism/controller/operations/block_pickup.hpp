/**
 * \file block_pickup.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/visitor/visitor.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/controller/operations/base_block_pickup.hpp"

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
 * \class block_pickup
 * \ingroup operations detail
 *
 * \brief Fired whenever a robot picks up a free block in the arena (i.e. one
 * that is not part of a cache).
 */
class block_pickup : public ccops::base_block_pickup {
 private:
  struct visit_typelist_impl {
    using controllers = controller::typelist;
    using fsms = rmpl::typelist<fsm::fcrw_bst_fsm>;
    using value = boost::mpl::joint_view<controllers::type, fsms::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  ~block_pickup(void) override = default;

  block_pickup(const block_pickup&) = delete;
  block_pickup& operator=(const block_pickup&) = delete;

  /* Single target construction */
  void visit(controller::fcrw_bst_controller& controller);
  void visit(fsm::fcrw_bst_fsm& fsm);

 protected:
  block_pickup(crepr::base_block3D* block,
                    const rtypes::type_uuid& robot_id,
                    const rtypes::timestep& t);

 private:
  using ccops::base_block_pickup::visit;
};

NS_END(detail);

using block_pickup_visitor = rpvisitor::filtered_visitor<detail::block_pickup>;

NS_END(operations, controller, prism);

