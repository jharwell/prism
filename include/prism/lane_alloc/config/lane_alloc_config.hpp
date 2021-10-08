/**
 * \file lane_alloc_config.hpp
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
#include <string>

#include "rcppsw/config/base_config.hpp"
#include "rcppsw/types/timestep.hpp"

#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, lane_alloc, config);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * \struct lane_alloc_config
 * \ingroup lane_alloc config
 *
 * \brief Parsed configuration for  \ref lane_alloc::lane_allocator objects
 */
struct lane_alloc_config final : public rconfig::base_config {
  /**
   * \brief The lane allocation policy to use.
   */
  std::string policy{};

  /**
   * \brief The interference window to use (only applicable to
   * interference-related policies).
   */
  rtypes::timestep interference_window{rtypes::timestep(0)};
};

NS_END(config, controler, prism);

