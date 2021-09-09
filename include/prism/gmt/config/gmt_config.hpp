/**
 * \file gmt_config.hpp
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

#ifndef INCLUDE_PRISM_GMT_CONFIG_GMT_CONFIG_HPP_
#define INCLUDE_PRISM_GMT_CONFIG_GMT_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/config/base_config.hpp"

#include "prism/prism.hpp"
#include "prism/gmt/config/spct_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, config);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * \struct gmt_config
 * \ingroup gmt config
 *
 * \brief Configuration for the various graphs to be manipulated (constructed or
 * deconstructed).
 */
struct gmt_config final : public rconfig::base_config {
  std::vector<spct_config> spct{};
};

NS_END(config, gmt, prism);

#endif /* INCLUDE_PRISM_GMT_CONFIG_GMT_CONFIG_HPP_ */
