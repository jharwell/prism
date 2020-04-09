/**
 * \file env_dynamics_parser.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "silicon/support/tv/config/xml/env_dynamics_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, support, tv, config, xml);

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

    if (nullptr != bnode.FirstChild("manipulation_penalty", false)) {
      m_block_manip.parse(node_get(bnode, "manipulation_penalty"));
      m_config->block_manip_penalty =
          *m_block_manip
               .config_get<rct::config::xml::waveform_parser::config_type>();
    }
    if (nullptr != bnode.FirstChild("carry_throttle", false)) {
      m_block_carry.parse(node_get(bnode, "carry_throttle"));
      auto config =
          m_block_carry
              .config_get<rct::config::xml::waveform_parser::config_type>();
      m_config->rda.motion_throttle = *config;
    }
  }
} /* parse() */

bool env_dynamics_parser::validate(void) const {
  return m_block_manip.validate() && m_block_carry.validate();
} /* validate() */

NS_END(xml, config, tv, support, silicon);
