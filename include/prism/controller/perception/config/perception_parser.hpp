/**
 * \file perception_parser.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_PRISM_CONTROLLER_PERCEPTION_CONFIG_PERCEPTION_PARSER_HPP_
#define INCLUDE_PRISM_CONTROLLER_PERCEPTION_CONFIG_PERCEPTION_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "rcppsw/config/xml/xml_config_parser.hpp"

#include "prism/controller/perception/config/perception_config.hpp"
#include "cosm/subsystem/perception/config/xml/rlos_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, controller, perception, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class perception_parser
 * \ingroup controller perception config
 *
 * \brief Parses XML parameters for various perception subsystems into
 * \ref perception_config.
 */
class perception_parser final : public rconfig::xml::xml_config_parser {
 public:
  using config_type = perception_config;

  /**
   * \brief The root tag that all perception  parameters should lie under in
   * the XML tree.
   */
  inline static std::string kXMLRoot = "perception";

  bool validate(void) const override RCPPSW_ATTR(pure, cold);
  void parse(const ticpp::Element& node) override RCPPSW_COLD;

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  cspcxml::rlos_parser         m_rlos{};
  /* clang-format on */
};

NS_END(config, perception, controller, prism);

#endif /* INCLUDE_PRISM_CONTROLLER_CONFIG_PERCEPTION_PERCEPTION_PARSER_HPP_ */
