/**
 * \file gmt_parser.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/gmt/config/xml/gmt_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, gmt, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void gmt_parser::parse(const ticpp::Element& node) {
  ticpp::Element cnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  for (ticpp::Iterator<ticpp::Element> node_it = cnode.FirstChildElement();
       node_it != node_it.end();
       ++node_it) {
    if (node_it->Value() == spct_parser::kXMLRoot) {
      m_spct.parse(*node_it);
      m_validations.push_back(m_spct.validate());
      m_config->spct.push_back(*m_spct.config_get<spct_parser::config_type>());
    } else {
      ER_FATAL_SENTINEL("Unknown graph manipulation target type: %s",
                        node_it->Value().c_str());
    }
  } /* for(node_it..) */
} /* parse() */

bool gmt_parser::validate(void) const {
  for (size_t i = 0; i < m_validations.size(); ++i) {
    ER_CHECK(m_validations[i], "Parsed GMT %zu invalid", i);
  } /* for(i..) */

  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, gmt, prism);
