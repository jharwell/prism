/**
 * \file arena_block_pickup_interactor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "cosm/argos/interactors/free_block_pickup.hpp"
#include "cosm/tv/temporal_penalty.hpp"

#include "prism/fsm/construction_acq_goal.hpp"
#include "prism/metrics/blocks/block_manip_events.hpp"
#include "prism/support/tv/block_op_src.hpp"
#include "prism/support/tv/op_filter_status.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class arena_block_pickup_interactor
 * \ingroup support
 *
 * \brief Handle's a robot's (possible) free block pickup event on a given
 * timestep.
 */
template <typename TController, typename TControllerSpecMap>
class arena_block_pickup_interactor final
    : public cainteractors::free_block_pickup<TController,
                                              TControllerSpecMap> {
 public:
  using typename cainteractors::
      free_block_pickup<TController, TControllerSpecMap>::arena_map_type;
  using typename cainteractors::free_block_pickup<
      TController,
      TControllerSpecMap>::penalty_handler_type;

  arena_block_pickup_interactor(arena_map_type* const map,
                                argos::CFloorEntity* const floor,
                                penalty_handler_type* const handler)
      : cainteractors::free_block_pickup<TController, TControllerSpecMap>(
            map,
            floor,
            handler) {}

  arena_block_pickup_interactor(arena_block_pickup_interactor&&) = default;

  /* Not copy-constructible/assignable by default. */
  arena_block_pickup_interactor(const arena_block_pickup_interactor&) = delete;
  arena_block_pickup_interactor&
  operator=(const arena_block_pickup_interactor&) = delete;

  tv::op_filter_status
  robot_penalty_init(const TController& controller,
                     const rtypes::timestep& t,
                     penalty_handler_type* handler) override {
    return handler->penalty_init(controller, t, tv::block_op_src::ekARENA_PICKUP);
  }

  bool robot_goal_acquired(const TController& controller) const override {
    return controller.goal_acquired() &&
           fsm::construction_acq_goal::ekFORAGING_BLOCK ==
               controller.acquisition_goal();
  }

  void robot_previsit_hook(TController& controller,
                           const ctv::temporal_penalty& penalty) const override {
    /*
     * Penalty served needs to be set here rather than in the free block pickup
     * event, because the penalty is generic, and the event handles concrete
     * classes--no clean way to mix the two.
     */
    controller.block_manip_recorder()->record(
        metrics::blocks::block_manip_events::ekARENA_PICKUP, penalty.penalty());
  }
};

NS_END(support, prism);
