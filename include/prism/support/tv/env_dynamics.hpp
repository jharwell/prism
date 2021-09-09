/**
 * \file env_dynamics.hpp
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

#ifndef INCLUDE_PRISM_SUPPORT_TV_ENV_DYNAMICS_HPP_
#define INCLUDE_PRISM_SUPPORT_TV_ENV_DYNAMICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <map>
#include <utility>

#include "rcppsw/er/client.hpp"

#include "cosm/tv/temporal_penalty_handler.hpp"
#include "cosm/tv/metrics/base_env_dynamics_metrics.hpp"
#include "cosm/pal/tv/argos_rda_adaptor.hpp"
#include "cosm/tv/env_dynamics.hpp"

#include "prism/support/tv/block_op_src.hpp"
#include "prism/support/tv/block_op_penalty_handler.hpp"
#include "prism/controller/constructing_controller.hpp"
#include "prism/support/tv/metrics/env_dynamics_metrics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace prism::support::tv::config { struct env_dynamics_config; }
namespace cosm::pal {
class argos_controllerQ3D_adaptor;
}

NS_START(prism, support);

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
  rtypes::timestep ct_block_manip_penalty(void) const override;

  /* COSM environment dynamics overrides */
  void register_controller(const cpal::argos_controllerQ3D_adaptor& c) override;
  void unregister_controller(const cpal::argos_controllerQ3D_adaptor& c) override;
  bool penalties_flush(const cpal::argos_controllerQ3D_adaptor& c) override;

  /**
   * \brief Register the handlers from a \ref gmt::spc_gmt object with
   * the environment dynamics, one per construction lane.
   *
   * This cannot be done by the environment dynamics, because it cannot know in
   * general how many structures will be present in the environment, or what
   * their IDs will be.
   */
  void ct_placement_handler_register(const rtypes::type_uuid& target_id,
                                     std::unique_ptr<block_op_penalty_handler> handler) {
    m_ct_placement.emplace(std::make_pair(target_id, std::move(handler)));
    ER_INFO("Registered placement handler for target%s",
            rcppsw::to_string(target_id).c_str());
  }

  /**
   * \brief Update the state of applied variances. Should be called once per
   * timestep.
   */
  void update(const rtypes::timestep& t);

  const rda_adaptor_type* rda_adaptor(void) const { return &m_rda; }

  const block_op_penalty_handler* penalty_handler(const block_op_src& src) const {
    return const_cast<env_dynamics*>(this)->penalty_handler_impl(src);
  }


  /**
   * \brief Return non-owning reference to a penalty handler for the specified
   * type of block operation; scope of usage must not exceed that of the
   * instance of this class used to generate the reference.
   */
  block_op_penalty_handler* penalty_handler(const block_op_src& src) {
    return penalty_handler_impl(src);
  }

  /**
   * \brief Return non-owning reference to a penalty handler for the specified
   * type of block operation for the specified construction target. Return
   * non-owning reference to the handler vector; scope of usage must not exceed
   * that of the instance of this class used to generate the reference.
   */
  const block_op_penalty_handler* ct_penalty_handler(const block_op_src& src,
                                                     const rtypes::type_uuid& id) const {
    return const_cast<env_dynamics*>(this)->penalty_handler_impl(src, id);
  }
  block_op_penalty_handler* ct_penalty_handler(const block_op_src& src,
                                               const rtypes::type_uuid& id) {
    return penalty_handler_impl(src, id);
  }

  /**
   * \brief Get the full list of all possible penalty handlers.
   *
   * This is a flatten hierarchy of:
   *
   * - Free block pickup handler
   * - All the handlers that are associated with all construction targets.
   */
  const_penalty_handlers all_penalty_handlers(void) const {
    const_penalty_handlers ret;
    ret.push_back(penalty_handler(block_op_src::ekARENA_PICKUP));
    for (const auto &pair : m_ct_placement) {
      ret.push_back(pair.second.get());
    } /* for(&pair..) */
    return ret;
  }


 private:
  using ct_placement_handler_map = std::map<rtypes::type_uuid,
                                            std::unique_ptr<block_op_penalty_handler>>;

  block_op_penalty_handler* penalty_handler_impl(
      const block_op_src& src) {
    switch (src) {
      case block_op_src::ekARENA_PICKUP:
        return &m_fb_pickup;
      default:
        ER_FATAL_SENTINEL("Bad penalty source %d", static_cast<int>(src));
    } /* switch() */
    return nullptr;
  }

  block_op_penalty_handler* penalty_handler_impl(const block_op_src& src,
                                                 const rtypes::type_uuid& target_id) {
    switch (src) {
      case block_op_src::ekCT_BLOCK_MANIP:
        {
          auto it = m_ct_placement.find(target_id);
          if (m_ct_placement.end() == it) {
            return nullptr;
          }
          return it->second.get();
        }
      default:
        ER_FATAL_SENTINEL("Bad penalty source %d", static_cast<int>(src));
    } /* switch() */
    return nullptr;
  }
  penalty_handlers all_penalty_handlers(void) {
    penalty_handlers ret;
    ret.push_back(penalty_handler(block_op_src::ekARENA_PICKUP));
    for (auto &pair : m_ct_placement) {
      ret.push_back(pair.second.get());
    } /* for(&pair..) */
    return ret;
  }

  /* clang-format off */
  rtypes::timestep         m_timestep{rtypes::timestep(0)};
  rda_adaptor_type         m_rda;
  block_op_penalty_handler m_fb_pickup;
  ct_placement_handler_map m_ct_placement{};
  /* clang-format on */
};

NS_END(tv, support, prism);

#endif /* INCLUDE_PRISM_SUPPORT_TV_ENV_DYNAMICS_HPP_ */
