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
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(node, m_config, anchor);
  XML_PARSE_ATTR(node, m_config, orientation);

  m_grid.parse(node_get(node, cds::config::xml::grid3D_parser::kXMLRoot));

  m_config->bounding_box = *m_grid.config_get<
    cds::config::xml::grid3D_parser::config_type>();

  if (nullptr != node.FirstChild("cube_blocks", false)) {
    auto cube_blocks = node_get(node, "cube_blocks");
    ticpp::Iterator<ticpp::Element> node_it;

    for (node_it = cube_blocks.FirstChildElement(); node_it != node_it.end();
         ++node_it) {
      rmath::vector3z tmp;
      node_attr_get(*node_it, "cell", tmp);
      m_config->cube_blocks[tmp] = {tmp};
    } /* for(node_it..) */
  }

  if (nullptr != node.FirstChild("ramp_blocks", false)) {
    auto ramp_blocks = node_get(node, "ramp_blocks");
    ticpp::Iterator<ticpp::Element> node_it;

    for (node_it = ramp_blocks.FirstChildElement(); node_it != node_it.end();
         ++node_it) {
      rmath::vector3z tmp;
      node_attr_get(*node_it, "cell", tmp);
      m_config->ramp_blocks[tmp] = {tmp, m_config->orientation};
    } /* for(node_it..) */
  }
} /* parse() */

NS_END(xml, config, structure, silicon);
