/**
 * \file lane_alloc_parser.hpp
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
#include <array>

#include "prism/lane_alloc/config/lane_alloc_config.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, lane_alloc, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class lane_alloc_parser
 * \ingroup lane_alloc config xml
 *
 * \brief Parses XML parameters \ref lane_alloc::lane_allocator objects.
 */
class lane_alloc_parser final : public rer::client<lane_alloc_parser>,
                                public rconfig::xml::xml_config_parser {
 public:
  using config_type = lane_alloc_config;

  /**
   * \brief The root tag that all \ref construction_lane_allocator parameters
   * should lie under in the XML tree.
   */
  inline static const std::string kXMLRoot = "lane_alloc";

  lane_alloc_parser(void)
      : ER_CLIENT_INIT("prism.lane_alloc.config.xml.lane_alloc_parser") {}

  void parse(const ticpp::Element& node) override RCPPSW_COLD;

  RCPPSW_NODISCARD std::string xml_root(void) const override RCPPSW_COLD {
    return kXMLRoot;
  }

 private:
  const rconfig::base_config* config_get_impl(void) const override RCPPSW_COLD {
    return m_config.get();
  }
  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

NS_END(xml, config, lane_alloc, prism);
