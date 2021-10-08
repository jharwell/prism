/**
 * \file visualization_parser.hpp
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
#include <memory>
#include <string>

#include "cosm/argos/vis/config/xml/visualization_parser.hpp"

#include "prism/prism.hpp"
#include "prism/support/config/visualization_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, support, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class visualization_parser
 * \ingroup support config xml
 *
 * \brief Parses XML parameters relating to visualization in loop functions into
 * \ref visualization_config.
 */
class visualization_parser final : public rer::client<visualization_parser>,
                                   public rconfig::xml::xml_config_parser {
 public:
  using config_type = visualization_config;

  /**
   * \brief The root tag that all visualization loop functions parameters should
   * lie under in the XML tree.
   */
  inline static const std::string kXMLRoot = "visualization";

  visualization_parser(void)
      : ER_CLIENT_INIT("prism.support.config.xml.visualization_parser") {}

  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type>             m_config{nullptr};
  cavis::config::xml::visualization_parser m_parent{};
  /* clang-format on */
};

NS_END(xml, config, support, prism);
