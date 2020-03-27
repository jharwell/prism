/**
 * \file silicon_pd_adaptor.cpp
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
#include "silicon/support/tv/silicon_pd_adaptor.hpp"

#include "cosm/arena/operations/free_block_drop.hpp"
#include "cosm/arena/base_arena_map.hpp"

#include "silicon/controller/constructing_controller.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, support, tv);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void silicon_pd_adaptor::pre_kill_cleanup(
    cpal::argos_controllerQ3D_adaptor* controller) {
  auto* constructing = static_cast<controller::constructing_controller*>(controller);
  /*
   * If the robot is carrying a block, drop/distribute it in the arena to avoid
   * it getting permanently lost when the it is removed.
   */
  if (constructing->is_carrying_block()) {
    ER_INFO("Kill victim robot %s is carrying block%d",
            constructing->GetId().c_str(),
            constructing->block()->id().v());
    auto it = std::find_if(arena_map()->blocks().begin(),
                           arena_map()->blocks().end(),
                           [&](const auto& b) {
                             return constructing->block()->id() == b->id();
                           });
    /*
     * We are not REALLY holding all the arena map locks, but since population
     * dynamics are always applied AFTER all robots have had their control steps
     * run, we are in a non-concurrent context, so no reason to grab them.
     */
    caops::free_block_drop_visitor adrop_op(
        *it,
        rmath::dvec2uvec(constructing->pos2D(), arena_map()->grid_resolution().v()),
        arena_map()->grid_resolution(),
        carena::arena_map_locking::ekALL_HELD);

    adrop_op.visit(*arena_map());
  }
} /* pre_kill_cleanup() */

NS_END(tv, support, silicon);
