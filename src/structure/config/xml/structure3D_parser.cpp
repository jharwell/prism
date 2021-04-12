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
  /* We do not call get_node() before parsing--a rare exception */
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(node, m_config, anchor);
  XML_PARSE_ATTR(node, m_config, orientation);

  /*
   * Since we don't know (for sure), and in general should not rely upon, the #
   * of decimal places used in the specification for structure orientation in
   * the input file, we round it to the nearest cardinal direction, if it is
   * specified with a reasonable # of decimals and is close enough to one.
   */
  if (rmath::radians::kZERO.is_equal(m_config->orientation, 0.001)) {
    m_config->orientation = rmath::radians::kZERO;
  } else if (rmath::radians::kPI_OVER_TWO.is_equal(m_config->orientation, 0.001)) {
    m_config->orientation = rmath::radians::kPI_OVER_TWO;
  } else if (rmath::radians::kPI.is_equal(m_config->orientation, 0.001)) {
    m_config->orientation = rmath::radians::kPI;
  } else if (rmath::radians::kTHREE_PI_OVER_TWO.is_equal(m_config->orientation, 0.001)) {
    m_config->orientation = rmath::radians::kTHREE_PI_OVER_TWO;
  }

  m_grid.parse(node);

  m_config->bounding_box =
      *m_grid.config_get<cds::config::xml::grid3D_parser::config_type>();

  if (nullptr != node.FirstChild("cube_blocks", false)) {
    auto cube_blocks = node_get(node, "cube_blocks");
    ticpp::Iterator<ticpp::Element> node_it;

    for (node_it = cube_blocks.FirstChildElement(); node_it != node_it.end();
         ++node_it) {
      rmath::vector3z tmp;
      node_attr_get(*node_it, "cell", tmp);
      m_config->cube_blocks[tmp] = { tmp };
    } /* for(node_it..) */
  }

  if (nullptr != node.FirstChild("ramp_blocks", false)) {
    auto ramp_blocks = node_get(node, "ramp_blocks");
    ticpp::Iterator<ticpp::Element> node_it;

    for (node_it = ramp_blocks.FirstChildElement(); node_it != node_it.end();
         ++node_it) {
      rmath::vector3z tmp;
      node_attr_get(*node_it, "cell", tmp);
      m_config->ramp_blocks[tmp] = { tmp, m_config->orientation };
    } /* for(node_it..) */
  }
} /* parse() */

NS_END(xml, config, structure, silicon);
