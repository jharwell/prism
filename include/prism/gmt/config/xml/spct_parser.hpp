/**
 * \file spct_parser.hpp
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

#include "prism/gmt/config/spct_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, gmt, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class spct_parser
 * \ingroup gmt config xml
 *
 * \brief Parses XML parameters defining \ref spct to be constructed at
 * the start of simulation.
 */
class spct_parser final : public rer::client<spct_parser>,
                          public rconfig::xml::xml_config_parser {
 public:
  using config_type = spct_config;
  spct_parser(void) : ER_CLIENT_INIT("prism.gmt.config.xml.spct_parser") {}

  inline static const std::string kXMLRoot = "spct";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCSW_CONST;

  std::string xml_root(void) const override RCPPSW_COLD { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override RCPPSW_COLD {
    return m_config.get();
  }
  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

NS_END(xml, config, gmt, prism);
