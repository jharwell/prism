/**
 * \file constructing_controller_repository.hpp
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

#ifndef INCLUDE_SILICON_CONTROLLER_CONFIG_CONSTRUCTING_CONTROLLER_REPOSITORY_HPP_
#define INCLUDE_SILICON_CONTROLLER_CONFIG_CONSTRUCTING_CONTROLLER_REPOSITORY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/xml/xml_config_repository.hpp"

#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, controller, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class constructing_controller_repository
 * \ingroup controller config
 *
 * \brief Collection of all parameter parsers and parse results needed by
 * all constructing controllers.
 */
class constructing_controller_repository
    : public rconfig::xml::xml_config_repository {
 public:
  constructing_controller_repository(void) RCSW_COLD;
};

NS_END(config, controller, silicon);

#endif /* INCLUDE_SILICON_CONFIG_CONSTRUCTING_CONTROLLER_REPOSITORY_HPP_ */
