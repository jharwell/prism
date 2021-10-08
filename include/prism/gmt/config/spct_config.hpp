/**
 * \file spct_config.hpp
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
#include "rcppsw/math/vector3.hpp"

#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, config);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * \struct spct_config
 * \ingroup gmt config
 *
 * \brief Parsed configuration for the \ref spc_gmt object.
 */
struct spct_config final : public rconfig::base_config {
  rmath::vector3d anchor{};
  rmath::radians  orientation{};

  /**
   * \brief The bounding box for the target. Must be specified in grid units
   * (\ref rmath::vector3z) rather than real valued units (\ref
   * rmath::vector3d), because this is a structure generated to be agnostic to
   * the resolution used.
   */
  rmath::vector3z bounding_box{};
  std::string     graphml{};
};

NS_END(config, gmt, prism);

