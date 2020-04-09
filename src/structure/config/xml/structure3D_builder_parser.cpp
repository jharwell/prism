/**
 * \file structure3D_builder_parser.cpp
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
#include "silicon/structure/config/xml/structure3D_builder_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, structure, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void structure3D_builder_parser::parse(const ticpp::Element& node) {
  ticpp::Element bnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(bnode, m_config, build_src);
  XML_PARSE_ATTR_DFLT(bnode, m_config, static_build_interval, rtypes::timestep(1));
  XML_PARSE_ATTR_DFLT(bnode, m_config, static_build_interval_count, 1UL);
} /* parse() */

NS_END(xml, config, structure, silicon);
