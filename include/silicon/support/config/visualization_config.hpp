/**
 * \file visualization_config.hpp
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

#ifndef INCLUDE_SILICON_SUPPORT_CONFIG_VISUALIZATION_CONFIG_HPP_
#define INCLUDE_SILICON_SUPPORT_CONFIG_VISUALIZATION_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/vis/config/visualization_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, support, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct visualization_config
 * \ingroup support config
 *
 * \brief SILICON visualizations beyond those supported in COSM.
 */
struct visualization_config final : public cvconfig::visualization_config {
  bool robot_nearest_ct{false};
};

NS_END(config, support, silicon);

#endif /* INCLUDE_SILICON_SUPPORT_CONFIG_VISUALIZATION_CONFIG_HPP_ */
