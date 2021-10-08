/**
 * \file spct_parser.cpp
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
#include "prism/gmt/config/xml/spct_parser.hpp"
#include "prism/gmt/utils.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, gmt, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void spct_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s",
           node.Value().c_str(),
           kXMLRoot.c_str());

  /* We do not call get_node() before parsing--a rare exception */
  m_config = std::make_unique<config_type>();
  XML_PARSE_ATTR(node, m_config, anchor);
  XML_PARSE_ATTR(node, m_config, bounding_box);
  XML_PARSE_ATTR(node, m_config, graphml);

  std::string orientation;
  node_attr_get(node, "orientation", orientation);

  /*
   * We use strings rather than numerical radian values because that is much
   * clearer and less error prone, given that PRISM only supports the 4 cardinal
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

bool spct_parser::validate(void) const {
  RCPPSW_CHECK(orientation_valid(m_config->orientation));

  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, gmt, prism);
