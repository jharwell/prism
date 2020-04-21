/**
 * \file lane_alloc_parser.hpp
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

#ifndef INCLUDE_SILICON_CONTROLLER_CONFIG_XML_LANE_ALLOC_PARSER_HPP_
#define INCLUDE_SILICON_CONTROLLER_CONFIG_XML_LANE_ALLOC_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "silicon/controller/config/lane_alloc_config.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, controller, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class lane_alloc_parser
 * \ingroup config controller xml
 *
 * \brief Parses XML parameters defining \ref lane_alloc to be constructed at
 * the start of simulation.
 */
class lane_alloc_parser final : public rconfig::xml::xml_config_parser {
 public:
  using config_type = lane_alloc_config;

  /**
   * \brief The root tag that all \ref construction_lane_allocator parameters
   * should lie under in the XML tree.
   */
  static constexpr const char kXMLRoot[] = "lane_alloc";

  void parse(const ticpp::Element& node) override RCSW_COLD;

  std::string xml_root(void) const override RCSW_COLD { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override RCSW_COLD {
    return m_config.get();
  }
  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

NS_END(xml, config, controller, silicon);

#endif /* INCLUDE_SILICON_CONTROLLER_CONFIG_XML_LANE_ALLOC_PARSER_HPP_ */
