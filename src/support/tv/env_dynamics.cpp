/**
 * \file env_dynamics.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/support/tv/env_dynamics.hpp"

#include "cosm/pal/controller/controllerQ3D.hpp"

#include "prism/support/base_loop_functions.hpp"
#include "prism/support/tv/config/env_dynamics_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, support, tv);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
env_dynamics::env_dynamics(const tv::config::env_dynamics_config* const config,
                           const support::base_loop_functions* const lf,
                           carena::base_arena_map* const map)
    : ER_CLIENT_INIT("prism.support.tv.env_dynamics"),
      m_rda(&config->rda, lf),
      m_fb_pickup(map, &config->block_manip_penalty, "Free Block Pickup") {}

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
  return penalty_handler(block_op_src::ekARENA_PICKUP)->penalty_calc(m_timestep);
} /* arena_block_manip_penalty() */

rtypes::timestep env_dynamics::ct_block_manip_penalty(void) const {
  /*
   * Relies on homogeneously applied penalties for all block manipulation events
   * in all structures.
   */
  return ct_penalty_handler(block_op_src::ekCT_BLOCK_MANIP,
                            rtypes::type_uuid(0))->penalty_calc(m_timestep);
} /* ct_block_manip_penalty() */

void env_dynamics::register_controller(const cpcontroller::controllerQ3D& c) {
  m_rda.register_controller(c.entity_id());
} /* register_controller() */

void env_dynamics::unregister_controller(const cpcontroller::controllerQ3D& c) {
  m_rda.unregister_controller(c.entity_id());
  penalties_flush(c);
} /* unregister_controller() */

bool env_dynamics::penalties_flush(const cpcontroller::controllerQ3D& c) {
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

NS_END(tv, support, prism);
