/**
 * \file loop_function_repository.cpp
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
#include "silicon/support/config/xml/loop_function_repository.hpp"

#include "silicon/structure/config/xml/construct_targets_parser.hpp"
#include "silicon/structure/config/xml/structure3D_builder_parser.hpp"
#include "silicon/support/config/xml/visualization_parser.hpp"
#include "silicon/support/tv/config/xml/tv_manager_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, support, config, xml);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
loop_function_repository::loop_function_repository(void) noexcept {
  parser_register<ssconfig::xml::construct_targets_parser,
                  ssconfig::construct_targets_config>(
      ssconfig::xml::construct_targets_parser::kXMLRoot);
  parser_register<ssconfig::xml::structure3D_builder_parser,
                  ssconfig::structure3D_builder_config>(
      ssconfig::xml::structure3D_builder_parser::kXMLRoot);
  parser_register<sstv::config::xml::tv_manager_parser,
                  sstv::config::tv_manager_config>(
      sstv::config::xml::tv_manager_parser::kXMLRoot);

  /*
   * Unregister the COSM visualization parser--it doesn't have everything we
   * need.
   */
  parser_unregister<cvconfig::xml::visualization_parser,
                    support::config::visualization_config>(
      cvconfig::xml::visualization_parser::kXMLRoot);

  parser_register<support::config::xml::visualization_parser,
                  support::config::visualization_config>(
      support::config::xml::visualization_parser::kXMLRoot);
}

NS_END(xml, config, support, silicon);
