/**
 * \file lane_alloc_parser.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/lane_alloc/config/xml/lane_alloc_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, lane_alloc, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void lane_alloc_parser::parse(const ticpp::Element& node) {
  ticpp::Element lnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(lnode, m_config, policy);
  XML_PARSE_ATTR_DFLT(lnode, m_config, interference_window, rtypes::timestep(0));
} /* parse() */

NS_END(xml, config, lane_alloc, silicon);
