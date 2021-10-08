/**
 * \file loop_function_repository.cpp
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
#include "prism/support/config/xml/loop_function_repository.hpp"

#include "prism/gmt/config/xml/gmt_parser.hpp"
#include "prism/gmt/config/xml/spct_builder_parser.hpp"
#include "prism/support/config/xml/visualization_parser.hpp"
#include "prism/support/tv/config/xml/tv_manager_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, support, config, xml);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
loop_function_repository::loop_function_repository(void) noexcept {
  parser_register<pgconfig::xml::gmt_parser,
                  pgconfig::gmt_config>(
      pgconfig::xml::gmt_parser::kXMLRoot);
  parser_register<pgconfig::xml::spct_builder_parser,
                  pgconfig::spct_builder_config>(
      pgconfig::xml::spct_builder_parser::kXMLRoot);
  parser_register<pstv::config::xml::tv_manager_parser,
                  pstv::config::tv_manager_config>(
      pstv::config::xml::tv_manager_parser::kXMLRoot);

  /*
   * Unregister the COSM visualization parser--it doesn't have everything we
   * need.
   */
  parser_unregister<cavis::config::xml::visualization_parser,
                    support::config::visualization_config>(
                        cavis::config::xml::visualization_parser::kXMLRoot);

  /* Register the visualization parser for PRISM */
  parser_register<psupport::config::xml::visualization_parser,
                  psupport::config::visualization_config>(
      psupport::config::xml::visualization_parser::kXMLRoot);
}

NS_END(xml, config, support, prism);
