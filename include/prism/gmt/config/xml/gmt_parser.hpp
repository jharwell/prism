/**
 * \file gmt_parser.hpp
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
#include <vector>

#include "rcppsw/config/xml/xml_config_parser.hpp"
#include "rcppsw/er/client.hpp"

#include "prism/gmt/config/gmt_config.hpp"
#include "prism/gmt/config/xml/spct_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, gmt, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class gmt_parser
 * \ingroup config structure xml
 *
 * \brief Parses XML parameters defining one or more gmion targets into
 * \ref gmt.
 */
class gmt_parser final : public rer::client<gmt_parser>,
                         public rconfig::xml::xml_config_parser {
 public:
  using config_type = gmt_config;

  gmt_parser(void) : ER_CLIENT_INIT("prism.gmt.config.xml.gmt_parser") {}

  /**
   * \brief The root tag that all \ref gmt parameters should lie
   * under in the XML tree.
   */
  inline static const std::string kXMLRoot = "gmt";

  bool validate(void) const override RCPPSW_ATTR(pure, cold);
  void parse(const ticpp::Element& node) override RCPPSW_COLD;

  std::string xml_root(void) const override RCPPSW_COLD { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override RCPPSW_COLD {
    return m_config.get();
  }
  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  spct_parser                  m_spct{};
  std::vector<bool>            m_validations{};
  /* clang-format on */
};

NS_END(xml, config, gmt, prism);

