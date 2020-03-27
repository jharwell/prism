/**
 * \file construct_targets_config.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_CONFIG_CONSTRUCT_TARGETS_CONFIG_HPP_
#define INCLUDE_SILICON_STRUCTURE_CONFIG_CONSTRUCT_TARGETS_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/config/base_config.hpp"

#include "silicon/silicon.hpp"
#include "silicon/structure/config/structure3D_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, config);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * \struct construct_targets_config
 * \ingroup structure config
 *
 * \brief Configuration for the structures to be built during simulation.
 */
struct construct_targets_config : public rconfig::base_config {
  std::vector<structure3D_config> targets{};
};

NS_END(config, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_CONFIG_CONSTRUCT_TARGETS_CONFIG_HPP_ */
