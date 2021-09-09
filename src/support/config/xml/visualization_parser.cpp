/**
 * \file visualization_parser.cpp
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
#include "prism/support/config/xml/visualization_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, support, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void visualization_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ticpp::Element vnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    m_parent.parse(node);
    *static_cast<cvconfig::visualization_config*>(m_config.get()) =
        *m_parent.config_get<cvconfig::xml::visualization_parser::config_type>();

    XML_PARSE_ATTR_DFLT(vnode, m_config, robot_nearest_ct, false);
  }
} /* parse() */

NS_END(xml, config, support, prism);
