/**
 * \file structure3D_parser.cpp
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
#include "silicon/structure/config/xml/structure3D_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, structure, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void structure3D_parser::parse(const ticpp::Element& node) {
  ticpp::Element snode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(snode, m_config, anchor);
  XML_PARSE_ATTR(snode, m_config, bounding_box);
  XML_PARSE_ATTR(snode, m_config, orientation);

  ticpp::Iterator<ticpp::Element> node_it;
  auto cube_blocks = node_get(snode, "cube_blocks");
  auto ramp_blocks = node_get(snode, "ramp_blocks");

  for (node_it = cube_blocks.FirstChildElement();
       node_it != node_it.end();
       ++node_it) {
    rmath::vector3u tmp;
    node_attr_get(*node_it, "loc", tmp);
    m_config->cube_blocks.push_back(tmp);
  } /* for(node_it..) */

  for (node_it = ramp_blocks.FirstChildElement();
       node_it != node_it.end();
       ++node_it) {
    rmath::vector3u tmp;
    node_attr_get(*node_it, "loc", tmp);
    m_config->ramp_blocks.push_back(tmp);
  } /* for(node_it..) */
} /* parse() */

NS_END(xml, config, structure, silicon);
