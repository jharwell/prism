/**
 * \file perception_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_PRISM_CONTROLLER_PERCEPTION_CONFIG_PERCEPTION_CONFIG_HPP_
#define INCLUDE_PRISM_CONTROLLER_PERCEPTION_CONFIG_PERCEPTION_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/config/base_config.hpp"

#include "cosm/subsystem/perception/config/rlos_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, controller, perception, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct perception_config
 * \ingroup controller perception config
 *
 * \brief Configuration for robot perception.
 */
struct perception_config final : public rconfig::base_config {
  std::string type{""};

  cspconfig::rlos_config rlos {};
};

NS_END(config, perception, controller, prism);

#endif /* INCLUDE_PRISM_CONTROLLER_PERCEPTION_CONFIG_PERCEPTION_CONFIG_HPP_ */
