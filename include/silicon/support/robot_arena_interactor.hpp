/**
 * \file robot_arena_interactor.hpp
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

#ifndef INCLUDE_SILICON_SUPPORT_ROBOT_ARENA_INTERACTOR_HPP_
#define INCLUDE_SILICON_SUPPORT_ROBOT_ARENA_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/types/timestep.hpp"

#include "silicon/metrics/silicon_metrics_aggregator.hpp"
#include "silicon/silicon.hpp"
#include "silicon/support/arena_block_pickup_interactor.hpp"
#include "silicon/support/ct_block_place_interactor.hpp"
#include "silicon/support/interactor_status.hpp"
#include "silicon/support/mpl/arena_block_pickup_spec.hpp"
#include "silicon/support/mpl/ct_block_place_spec.hpp"
#include "silicon/support/tv/block_op_src.hpp"
#include "silicon/support/tv/env_dynamics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, support);

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
                         sstructure::ct_manager* manager,
                         argos::CFloorEntity* const floor,
                         tv::env_dynamics* const envd,
                         smetrics::silicon_metrics_aggregator* metrics_agg)
      : ER_CLIENT_INIT("silicon.support.robot_arena_interactor"),
        m_arena_pickup(map,
                       floor,
                       envd->penalty_handler(tv::block_op_src::ekARENA_PICKUP)),
        m_block_place(manager, map, metrics_agg, floor) {}

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
  arena_block_pickup_interactor<TControllerType, arena_pickup_spec> m_arena_pickup;
  ct_block_place_interactor<TControllerType, block_place_spec>      m_block_place;
  /* clang-format on */
};

NS_END(support, silicon);

#endif /* INCLUDE_SILICON_SUPPORT_ROBOT_ARENA_INTERACTOR_HPP_ */
