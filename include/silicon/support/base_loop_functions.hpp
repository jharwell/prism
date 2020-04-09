/**
 * \file base_loop_functions.hpp
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

#ifndef INCLUDE_SILICON_SUPPORT_BASE_LOOP_FUNCTIONS_HPP_
#define INCLUDE_SILICON_SUPPORT_BASE_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/config/rng_config.hpp"
#include "rcppsw/math/radians.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/utils/color.hpp"

#include "cosm/pal/argos_sm_adaptor.hpp"
#include "silicon/support/config/xml/loop_function_repository.hpp"
#include "silicon/support/tv/tv_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::metrics::config {
struct output_config;
} /* namespace cosm::metrics::config */

namespace silicon::support::tv::config {
struct tv_manager_config;
}

namespace silicon::structure {
namespace config {
struct structure3D_builder_config;
struct construct_targets_config;
} /* namespace config */

class structure3D_builder;
class structure3D;
} /* namespace silicon::structure */

NS_START(silicon, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class base_loop_functions
 * \ingroup support
 *
 * \brief The base loop functions in SILICON that all other loop functions
 * inherit from.
 *
 * This class is not a functional set of loop functions, but it provides
 * functions needed across multiple derived classes, but functionality that
 * could not just be free functions because they require access to members in
 * the \ref argos::CLoopFunctions class.
 */
class base_loop_functions : public cpal::argos_sm_adaptor,
                            public rer::client<base_loop_functions> {
 public:
  using arena_map_type = carena::base_arena_map<crepr::base_block3D>;

  base_loop_functions(void) RCSW_COLD;
  ~base_loop_functions(void) override RCSW_COLD;

  /* Not copy constructible/assignable by default */
  base_loop_functions(const base_loop_functions&) = delete;
  base_loop_functions& operator=(const base_loop_functions&) = delete;

  /* swarm manager overrides */
  void init(ticpp::Element&) override RCSW_COLD;
  void reset(void) override RCSW_COLD;
  void pre_step(void) override;
  void post_step(void) override;

  const tv::tv_manager* tv_manager(void) const { return m_tv_manager.get(); }
  const arena_map_type* arena_map(void) const {
    return argos_sm_adaptor::arena_map<arena_map_type>();
  }

 protected:
  tv::tv_manager* tv_manager(void) { return m_tv_manager.get(); }
  const config::xml::loop_function_repository* config(void) const {
    return &m_config;
  }
  config::xml::loop_function_repository* config(void) { return &m_config; }

  arena_map_type* arena_map(void) {
    return argos_sm_adaptor::arena_map<arena_map_type>();
  }
 private:
  using targets_vector_type = std::vector<std::unique_ptr<structure::structure3D>>;
  using builders_vector_type = std::vector<std::unique_ptr<structure::structure3D_builder>>;

  /**
   * \brief Initialize temporal variance handling.
   *
   * \param tvp Parsed TV parameters.
   */
  void tv_init(const tv::config::tv_manager_config* tvp) RCSW_COLD;

  /**
   * \brief Initialize logging for all support/loop function code.
   *
   * \param output Parsed output parameters.
   */
  void output_init(const cmconfig::output_config* output) RCSW_COLD;

  /**
   * \brief Initialize the \ref structure::structure3D_builder.
   *
   * \param builder_config Parsed builder parameters.
   * \param target_config Parsed \ref structure::structure3D parameters (one per
   *                      structure).
   */
  void construction_init(const ssconfig::structure3D_builder_config* builder_config,
                         const ssconfig::construct_targets_config* targets_config);

  /* clang-format off */
  config::xml::loop_function_repository m_config{};
  std::unique_ptr<tv::tv_manager>       m_tv_manager{nullptr};
  targets_vector_type                   m_targets{};
  builders_vector_type                  m_builders{};
  /* clang-format on */
};

NS_END(support, silicon);

#endif /* INCLUDE_SILICON_SUPPORT_BASE_LOOP_FUNCTIONS_HPP_ */
