/**
 * \file env_dynamics_parser.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "rcppsw/config/xml/xml_config_parser.hpp"

#include "cosm/tv/config/xml/temporal_penalty_parser.hpp"

#include "prism/support/tv/config/env_dynamics_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, support, tv, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class env_dynamics_parser
 * \ingroup support tv config xml
 *
 * \brief Parses XML parameters for \ref env_dynamics into \ref
 * env_dynamics_config.
 */
class env_dynamics_parser final : public rer::client<env_dynamics_parser>,
                                  public rconfig::xml::xml_config_parser {
 public:
  using config_type = env_dynamics_config;

  /**
   * \brief The root tag that all temporal variance parameters should lie under
   * in the XML tree.
   */
  inline static const std::string kXMLRoot = "env_dynamics";

  env_dynamics_parser(void);

  void parse(const ticpp::Element& node) override;
  bool validate(void) const override RCSW_CONST;

  std::string xml_root(void) const override { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type>              m_config{nullptr};
  ctv::config::xml::temporal_penalty_parser m_block_manip{};
  ctv::config::xml::temporal_penalty_parser m_block_carry{};
  /* clang-format on */
};

NS_END(xml, config, tv, support, prism);
