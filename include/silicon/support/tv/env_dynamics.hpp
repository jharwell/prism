/**
 * \file env_dynamics.hpp
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

#ifndef INCLUDE_SILICON_SUPPORT_TV_ENV_DYNAMICS_HPP_
#define INCLUDE_SILICON_SUPPORT_TV_ENV_DYNAMICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>

#include "rcppsw/er/client.hpp"

#include "cosm/tv/temporal_penalty_handler.hpp"
#include "cosm/tv/metrics/base_env_dynamics_metrics.hpp"
#include "cosm/pal/tv/argos_rda_adaptor.hpp"
#include "cosm/tv/env_dynamics.hpp"

#include "silicon/support/tv/block_op_src.hpp"
#include "silicon/support/tv/block_op_penalty_handler.hpp"
#include "silicon/controller/constructing_controller.hpp"
#include "silicon/support/tv/metrics/env_dynamics_metrics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::foraging::ds {
class base_arena_map;
} /* namespace cosm::foraging::ds */

namespace silicon::support::tv::config { struct env_dynamics_config; }
namespace cosm::pal {
class argos_controllerQ3D_adaptor;
}

NS_START(silicon, support);

class base_loop_functions;

NS_START(tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class env_dynamics
 * \ingroup support tv
 *
 * \brief Orchestrates all application of temporal variance in environmental
 * conditions to the swarm.
 */
class env_dynamics final : public rer::client<env_dynamics>,
                           public ctv::env_dynamics<cpal::argos_controllerQ3D_adaptor>,
                           public metrics::env_dynamics_metrics {
 public:
  using const_penalty_handlers = std::list<const ctv::temporal_penalty_handler*>;
  using penalty_handlers = std::list<ctv::temporal_penalty_handler*>;
  using rda_adaptor_type = cpal::tv::argos_rda_adaptor<controller::constructing_controller>;

  env_dynamics(const tv::config::env_dynamics_config * config,
               const support::base_loop_functions* lf,
               carena::base_arena_map* map);

  env_dynamics(const env_dynamics&) = delete;
  const env_dynamics& operator=(const env_dynamics&) = delete;

  /* environmental variance metrics */
  double avg_motion_throttle(void) const override;
  rtypes::timestep arena_block_manip_penalty(void) const override;
  rtypes::timestep structure_block_manip_penalty(void) const override;

  /* COSM env_dynamics overrides */
  void register_controller(const cpal::argos_controllerQ3D_adaptor& c) override;
  void unregister_controller(const cpal::argos_controllerQ3D_adaptor& c) override;
  bool penalties_flush(const cpal::argos_controllerQ3D_adaptor& c) override;

  /**
   * \brief Update the state of applied variances. Should be called once per
   * timestep.
   */
  void update(const rtypes::timestep& t);

  const rda_adaptor_type* rda_adaptor(void) const { return &m_rda; }

  /**
   * \brief Return non-owning reference to a penalty handler for the specified
   * type of block operation; scope of usage must not exceed that of the
   * instance of this class used to generate the reference.
   */
  const block_op_penalty_handler* penalty_handler(const block_op_src& src) const {
    return const_cast<env_dynamics*>(this)->penalty_handler(src);
  }
  block_op_penalty_handler* penalty_handler(const block_op_src& src) {
    switch (src) {
      case block_op_src::ekFREE_PICKUP:
        return &m_fb_pickup;
      case block_op_src::ekSTRUCTURE_PLACEMENT:
        return &m_structure_placement;
      default:
        ER_FATAL_SENTINEL("Bad penalty source %d", static_cast<int>(src));
    } /* switch() */
    return nullptr;
  }

  /**
   * \brief Get the full list of all possible penalty handlers. Note that for
   * some controller types there are handlers in the returned list for which a
   * controller can *NEVER* be serving a penalty for.
   */
  const_penalty_handlers all_penalty_handlers(void) const {
    return {penalty_handler(block_op_src::ekFREE_PICKUP),
          penalty_handler(block_op_src::ekSTRUCTURE_PLACEMENT),
          };
  }


 private:
  penalty_handlers all_penalty_handlers(void) {
    return {penalty_handler(block_op_src::ekFREE_PICKUP),
          penalty_handler(block_op_src::ekSTRUCTURE_PLACEMENT),
          };
  }

  /* clang-format off */
  rtypes::timestep         m_timestep{rtypes::timestep(0)};
  rda_adaptor_type         m_rda;
  block_op_penalty_handler m_fb_pickup;
  block_op_penalty_handler m_structure_placement;
  /* clang-format on */
};

NS_END(tv, support, silicon);

#endif /* INCLUDE_SILICON_SUPPORT_TV_ENV_DYNAMICS_HPP_ */
