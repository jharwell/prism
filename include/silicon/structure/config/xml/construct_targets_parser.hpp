/**
 * \file construct_targets_parser.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_CONFIG_XML_CONSTRUCT_TARGETS_PARSER_HPP_
#define INCLUDE_SILICON_STRUCTURE_CONFIG_XML_CONSTRUCT_TARGETS_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "silicon/structure/config/construct_targets_config.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"
#include "silicon/structure/config/xml/structure3D_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, structure, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class construct_targets_parser
 * \ingroup config structure xml
 *
 * \brief Parses XML parameters defining one or more construction targets into
 * \ref construct_targets.
 */
class construct_targets_parser final : public rconfig::xml::xml_config_parser {
 public:
  using config_type = construct_targets_config;

  /**
   * \brief The root tag that all \ref construct_targets parameters should lie
   * under in the XML tree.
   */
  static constexpr const char kXMLRoot[] = "construct_targets";

  void parse(const ticpp::Element& node) override RCSW_COLD;

  std::string xml_root(void) const override RCSW_COLD { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override RCSW_COLD {
    return m_config.get();
  }
  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  structure3D_parser           m_target{};

  /* clang-format on */
};

NS_END(xml, config, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_CONFIG_XML_CONSTRUCT_TARGETS_PARSER_HPP_ */
