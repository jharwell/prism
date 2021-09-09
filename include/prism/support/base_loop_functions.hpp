/**
 * \file base_loop_functions.hpp
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

#ifndef INCLUDE_PRISM_SUPPORT_BASE_LOOP_FUNCTIONS_HPP_
#define INCLUDE_PRISM_SUPPORT_BASE_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/config/rng_config.hpp"
#include "rcppsw/math/radians.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/utils/color.hpp"

#include "cosm/pal/argos_sm_adaptor.hpp"

#include "prism/gmt/config/gmt_config.hpp"
#include "prism/gmt/config/spct_builder_config.hpp"
#include "prism/support/config/xml/loop_function_repository.hpp"
#include "prism/support/tv/config/tv_manager_config.hpp"
#include "prism/support/tv/tv_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::pal::config {
struct output_config;
} /* namespace cosm::pal::config */

NS_START(prism, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class base_loop_functions
 * \ingroup support
 *
 * \brief The base loop functions in PRISM that all other loop functions
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
  base_loop_functions(void) RCPPSW_COLD;
  ~base_loop_functions(void) override RCPPSW_COLD;

  /* Not copy constructible/assignable by default */
  base_loop_functions(const base_loop_functions&) = delete;
  base_loop_functions& operator=(const base_loop_functions&) = delete;

  /* swarm manager overrides */
  void init(ticpp::Element&) override RCPPSW_COLD;
  void reset(void) override RCPPSW_COLD;
  void pre_step(void) override;
  void post_step(void) override;

  const tv::tv_manager* tv_manager(void) const { return m_tv_manager.get(); }

 protected:
  tv::tv_manager* tv_manager(void) { return m_tv_manager.get(); }
  const config::xml::loop_function_repository* config(void) const {
    return &m_config;
  }
  config::xml::loop_function_repository* config(void) { return &m_config; }

 private:
  /**
   * \brief Initialize temporal variance handling.
   *
   * \param tvp Parsed TV parameters.
   */
  void tv_init(const tv::config::tv_manager_config* tvp) RCPPSW_COLD;

  /**
   * \brief Initialize logging for all support/loop function code.
   *
   * \param output Parsed output parameters.
   */
  void output_init(const cpconfig::output_config* output) override RCPPSW_COLD;

  /* clang-format off */
  config::xml::loop_function_repository  m_config{};
  std::unique_ptr<tv::tv_manager>        m_tv_manager{nullptr};
  /* clang-format on */
};

NS_END(support, prism);

#endif /* INCLUDE_PRISM_SUPPORT_BASE_LOOP_FUNCTIONS_HPP_ */
