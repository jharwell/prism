/**
 * \file env_dynamics_config.hpp
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

#ifndef INCLUDE_PRISM_SUPPORT_TV_CONFIG_ENV_DYNAMICS_CONFIG_HPP_
#define INCLUDE_PRISM_SUPPORT_TV_CONFIG_ENV_DYNAMICS_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/tv/config/base_env_dynamics_config.hpp"
#include "cosm/tv/config/temporal_penalty_config.hpp"

#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, support, tv, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct env_dynamics_config
 * \ingroup support tv config
 *
 * \brief Configuration for the \ref env_dynamics.
 */
struct env_dynamics_config final : public ctv::config::base_env_dynamics_config {
  ctv::config::temporal_penalty_config block_manip_penalty{};
  ctv::config::temporal_penalty_config block_carry{};
  /* open to extension */
};

NS_END(config, tv, support, prism);

#endif /* INCLUDE_PRISM_SUPPORT_TV_CONFIG_ENV_DYNAMICS_CONFIG_HPP_ */
