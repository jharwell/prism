/**
 * \file env_dynamics.cpp
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
#include "silicon/support/tv/env_dynamics.hpp"

#include "silicon/support/tv/config/env_dynamics_config.hpp"
#include "cosm/pal/argos_controllerQ3D_adaptor.hpp"
#include "silicon/support/base_loop_functions.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, support, tv);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
env_dynamics::env_dynamics(const tv::config::env_dynamics_config* const config,
                           const support::base_loop_functions* const lf,
                           carena::base_arena_map<crepr::base_block3D>* const map)
    : ER_CLIENT_INIT("silicon.support.tv.env_dynamics"),
      m_rda(&config->rda, lf),
      m_fb_pickup(map, &config->block_manip_penalty, "Free Block Pickup"),
      m_structure_placement(map,
                            &config->block_manip_penalty,
                            "Structure Block Placement") {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void env_dynamics::update(const rtypes::timestep& t) {
  m_timestep = t;
  m_rda.update();
} /* update(0) */

double env_dynamics::avg_motion_throttle(void) const {
  return m_rda.avg_motion_throttle();
} /* avg_motion_throttle() */

rtypes::timestep env_dynamics::arena_block_manip_penalty(void) const {
  return penalty_handler(block_op_src::ekFREE_PICKUP)->penalty_calc(m_timestep);
} /* arena_block_manip_penalty() */

rtypes::timestep env_dynamics::structure_block_manip_penalty(void) const {
  return penalty_handler(block_op_src::ekSTRUCTURE_PLACEMENT)->penalty_calc(m_timestep);
} /* structure_block_manip_penalty() */

void env_dynamics::register_controller(const cpal::argos_controllerQ3D_adaptor& c) {
  m_rda.register_controller(c.entity_id());
} /* register_controller() */

void env_dynamics::unregister_controller(
    const cpal::argos_controllerQ3D_adaptor& c) {
  m_rda.unregister_controller(c.entity_id());
  penalties_flush(c);
} /* unregister_controller() */

bool env_dynamics::penalties_flush(const cpal::argos_controllerQ3D_adaptor& c) {
  bool aborted = false;
  for (auto& h : all_penalty_handlers()) {
    if (h->is_serving_penalty(c)) {
      ER_ASSERT(!aborted,
                "Controller serving penalties from more than one source");
      h->penalty_abort(c);
      aborted = true;
      ER_INFO("%s flushed from serving '%s' penalty",
              c.GetId().c_str(),
              h->name().c_str());
    }
  } /* for(&h..) */
  return aborted;
} /* penalties_flush() */

NS_END(tv, support, silicon);
