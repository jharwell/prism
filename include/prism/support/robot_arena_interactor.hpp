/**
 * \file robot_arena_interactor.hpp
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
#include "rcppsw/er/client.hpp"
#include "rcppsw/types/timestep.hpp"

#include "prism/metrics/prism_metrics_manager.hpp"
#include "prism/prism.hpp"
#include "prism/support/arena_block_pickup_interactor.hpp"
#include "prism/support/ct_block_place_interactor.hpp"
#include "prism/support/ct_complete_interactor.hpp"
#include "prism/support/interactor_status.hpp"
#include "prism/support/mpl/arena_block_pickup_spec.hpp"
#include "prism/support/mpl/ct_block_place_spec.hpp"
#include "prism/support/tv/block_op_src.hpp"
#include "prism/support/tv/env_dynamics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * \class robot_arena_interactor
 * \ingroup support
 *
 * \brief Handle's a robot's interactions with the environment on each timestep.
 *
 * Including:
 *
 */
template <typename TControllerType>
class robot_arena_interactor final
    : public rer::client<robot_arena_interactor<TControllerType>> {
 public:
  robot_arena_interactor(carena::base_arena_map* const map,
                         pgmt::ct_manager* ct_manager,
                         argos::CFloorEntity* const floor,
                         tv::env_dynamics* const envd,
                         pmetrics::prism_metrics_manager* metrics_manager)
      : ER_CLIENT_INIT("prism.support.robot_arena_interactor"),
        m_ct_complete(ct_manager),
        m_arena_pickup(map,
                       floor,
                       envd->penalty_handler(tv::block_op_src::ekARENA_PICKUP)),
        m_block_place(ct_manager, map, metrics_manager, floor) {}

  robot_arena_interactor(robot_arena_interactor&&) = default;

  /* not copy constructible/assignable by default */
  robot_arena_interactor(const robot_arena_interactor&) = delete;
  robot_arena_interactor& operator=(const robot_arena_interactor&) = delete;

  /**
   * \brief The actual handling function for the interactions.
   *
   * \param controller The controller to handle interactions for.
   * \param t The current timestep.
   */
  interactor_status operator()(TControllerType& controller,
                               const rtypes::timestep& t) {
    auto status = m_ct_complete(controller);

    /* nothing more to do */
    if (interactor_status::ekROBOT_STOPPED == status) {
      return status;
    }

    if (controller.is_carrying_block()) {
      return m_block_place(controller, t);
    } else { /* The robot has no block item */
      return m_arena_pickup(controller, t);
    }
  }

 private:
  using arena_pickup_spec =
      typename mpl::arena_block_pickup_spec<controller::typelist>;
  using block_place_spec =
      typename mpl::ct_block_place_spec<controller::typelist>;

  /* clang-format off */
  ct_complete_interactor<TControllerType, arena_pickup_spec>        m_ct_complete;
  arena_block_pickup_interactor<TControllerType, arena_pickup_spec> m_arena_pickup;
  ct_block_place_interactor<TControllerType, block_place_spec>      m_block_place;
  /* clang-format on */
};

NS_END(support, prism);

