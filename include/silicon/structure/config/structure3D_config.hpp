/**
 * \file structure3D_config.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_CONFIG_STRUCTURE3D_CONFIG_HPP_
#define INCLUDE_SILICON_STRUCTURE_CONFIG_STRUCTURE3D_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <string>

#include "rcppsw/config/base_config.hpp"
#include "rcppsw/math/vector3.hpp"

#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, config);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
struct structure3D_config : public rconfig::base_config {
  rmath::vector3u              anchor{};
  rmath::vector3u              bounding_box{};
  std::string                  orientation{};

  std::vector<rmath::vector3u> cube_blocks{};
  std::vector<rmath::vector3u> ramp_blocks{};
};

NS_END(config, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_CONFIG_STRUCTURE3D_CONFIG_HPP_ */
