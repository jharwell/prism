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
#include <map>
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
struct ramp_block_loc_spec {
  rmath::vector3z loc{};
  rmath::radians z_rotation{};
};

struct cube_block_loc_spec {
  rmath::vector3z loc{};
};

/**
 * \struct structure3D_config
 * \ingroup structure config
 *
 * \brief Parsed configuration for the \ref structure3D object.
 */
struct structure3D_config final : public rconfig::base_config {
  rmath::vector3d              anchor{};
  rmath::vector3z              bounding_box{};
  rmath::radians               orientation{};
  std::string                  id{};

  /*
   * We use maps with somewhat redundant (key, value) pairs in order to make the
   * process of determining "what kind of block should this (i,j,k) location
   * contain when the structure is completed?" efficient. Just using vectors
   * requires a linear scan for every single passed location.
   */
  std::map<rmath::vector3z, cube_block_loc_spec> cube_blocks{};
  std::map<rmath::vector3z, ramp_block_loc_spec> ramp_blocks{};
};

NS_END(config, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_CONFIG_STRUCTURE3D_CONFIG_HPP_ */
