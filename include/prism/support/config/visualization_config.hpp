/**
 * \file visualization_config.hpp
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
#include "cosm/argos/vis/config/visualization_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, support, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct visualization_config
 * \ingroup support config
 *
 * \brief PRISM visualizations beyond those supported in COSM.
 */
struct visualization_config final : public cavis::config::visualization_config {
  bool robot_nearest_ct{false};
};

NS_END(config, support, prism);
