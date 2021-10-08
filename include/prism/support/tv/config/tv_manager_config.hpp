/**
 * \file tv_manager_config.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "cosm/tv/config/population_dynamics_config.hpp"
#include "prism/support/tv/config/env_dynamics_config.hpp"

#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, support, tv, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct tv_manager_config
 * \ingroup support tv config
 *
 * \brief Configuration for the \ref tv_manager.
 */
struct tv_manager_config final : public rconfig::base_config {
  tv::config::env_dynamics_config env_dynamics{};
  ctv::config::population_dynamics_config population_dynamics{};
};

NS_END(config, tv, support, prism);

