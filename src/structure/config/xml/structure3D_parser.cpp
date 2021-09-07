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
#include "silicon/structure/utils.hpp"

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
  XML_PARSE_ATTR(node, m_config, bounding_box);
  XML_PARSE_ATTR(node, m_config, graphml);

  std::string orientation;
  node_attr_get(node, "orientation", orientation);

  /*
   * We use strings rather than numerical radian values because that is much
   * clearer and less prone, given that SILICON only supports the 4 cardinal
   * directions anyway.
   */
  if ("0" == orientation) {
    m_config->orientation = rmath::radians::kZERO;
  } else if ("PI/2" == orientation) {
    m_config->orientation = rmath::radians::kPI_OVER_TWO;
  } else if ("PI" == orientation) {
    m_config->orientation = rmath::radians::kPI;
  } else if ("3PI/2" == orientation) {
    m_config->orientation = rmath::radians::kTHREE_PI_OVER_TWO;
  } else {
    m_config->orientation = rmath::radians(-1);
  }
} /* parse() */

bool structure3D_parser::validate(void) const {
  RCPPSW_CHECK(orientation_valid(m_config->orientation));

  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, structure, silicon);
