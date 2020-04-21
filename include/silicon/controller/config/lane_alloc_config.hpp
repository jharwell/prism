/**
 * \file lane_alloc_config.hpp
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

#ifndef INCLUDE_SILICON_CONTROLLER_CONFIG_LANE_ALLOC_CONFIG_HPP_
#define INCLUDE_SILICON_CONTROLLER_CONFIG_LANE_ALLOC_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/config/base_config.hpp"

#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, controller, config);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * \struct lane_alloc_config
 * \ingroup controller config
 *
 * \brief Parsed configuration for  \ref construction_lane_allocator objects
 */
struct lane_alloc_config final : public rconfig::base_config {
  std::string policy{};
};

NS_END(config, controler, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_CONFIG_LANE_ALLOC_CONFIG_HPP_ */
