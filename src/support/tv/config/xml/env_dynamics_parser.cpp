/**
 * \file env_dynamics_parser.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "prism/support/tv/config/xml/env_dynamics_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, support, tv, config, xml);
using temporal_config_type =
    ctv::config::xml::temporal_penalty_parser::config_type;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
env_dynamics_parser::env_dynamics_parser(void) {
  m_block_manip.xml_root("manipulation_penalty");
  m_block_carry.xml_root("carry_throttle");
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void env_dynamics_parser::parse(const ticpp::Element& node) {
  /* No environmental dynamics configured */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ticpp::Element tvnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  /* block dynamics configured */
  if (nullptr != tvnode.FirstChild("blocks", false)) {
    ticpp::Element bnode = node_get(tvnode, "blocks");

    m_block_manip.parse(bnode);
    if (m_block_manip.is_parsed()) {
      m_config->block_manip_penalty =
          *m_block_manip.config_get<temporal_config_type>();
    }

    m_block_carry.parse(bnode);
    if (m_block_carry.is_parsed()) {
      m_config->rda.motion_throttle =
          m_block_carry.config_get<temporal_config_type>()->waveform;
    }
  }
} /* parse() */

bool env_dynamics_parser::validate(void) const {
  return m_block_manip.validate() && m_block_carry.validate();
} /* validate() */

NS_END(xml, config, tv, support, prism);
