/**
 * \file block_op_penalty_handler.hpp
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

#ifndef INCLUDE_SILICON_SUPPORT_TV_BLOCK_OP_PENALTY_HANDLER_HPP_
#define INCLUDE_SILICON_SUPPORT_TV_BLOCK_OP_PENALTY_HANDLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <boost/optional.hpp>

#include "rcppsw/types/type_uuid.hpp"

#include "cosm/tv/temporal_penalty_handler.hpp"
#include "cosm/foraging/tv/penalty_id_calculator.hpp"

#include "silicon/support/tv/block_op_filter.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, support, tv);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class block_op_penalty_handler
 * \ingroup support tv
 *
 * \brief The handler for block operation penalties for robots (e.g. picking
 * up, placing on the \ref structure3D).
 */
class block_op_penalty_handler final : public ctv::temporal_penalty_handler,
                                       public rer::client<block_op_penalty_handler> {
 public:
  block_op_penalty_handler(carena::base_arena_map<crepr::base_block3D>* const map,
                           const rct::config::waveform_config* const config,
                           const std::string& name)
      : temporal_penalty_handler(config, name),
        ER_CLIENT_INIT("silicon.support.tv.block_op_penalty_handler"),
        mc_map(map),
        m_filter(mc_map) {}

  ~block_op_penalty_handler(void) override = default;
  block_op_penalty_handler& operator=(const block_op_penalty_handler& other) =
      delete;
  block_op_penalty_handler(const block_op_penalty_handler&) = delete;

  /**
   * \brief Check if a robot has acquired a block or is in the nest, and is
   * trying to drop/pickup a block. If so, create a \ref temporal_penalty object
   * and associate it with the robot.
   *
   * \tparam TControllerType The type of the controller. Must be a template
   * parameter, because of the goal acquisition determination done by \ref
   * block_op_filter.
   *
   * \param controller The robot to check.
   * \param src The penalty source (i.e. what event caused this penalty to be
   *            applied).
   * \param t The current timestep.
   * \param prox_dist The minimum distance that the cache site needs to be from
   *                  all caches in the arena.
   */
  template<typename TControllerType>
  op_filter_status penalty_init(const TControllerType& controller,
                                const rtypes::timestep& t,
                                block_op_src src) {
    auto filter = m_filter(controller, src);
    if (filter != op_filter_status::ekSATISFIED) {
      return filter;
    }
    ER_ASSERT(!is_serving_penalty(controller),
              "%s already serving block penalty?",
              controller.GetId().c_str());

    rtypes::type_uuid id = m_id_calc(controller, src);
    rtypes::timestep orig_duration = penalty_calc(t);
    auto duration RCSW_UNUSED = penalty_add(&controller,
                                            id,
                                            orig_duration,
                                            t);

    ER_INFO("%s: block%d start=%u, penalty=%u, adjusted penalty=%u src=%d",
            controller.GetId().c_str(),
            id.v(),
            t.v(),
            orig_duration.v(),
            duration.v(),
            static_cast<int>(src));

    return filter;
  }

 private:
  /* clang-format off */
  const carena::base_arena_map<crepr::base_block3D>* const mc_map;

  block_op_filter                                          m_filter;
  cforaging::tv::penalty_id_calculator                     m_id_calc{};
  /* clang-format on */
};
NS_END(tv, support, silicon);

#endif /* INCLUDE_SILICON_SUPPORT_TV_BLOCK_OP_PENALTY_HANDLER_HPP_ */
