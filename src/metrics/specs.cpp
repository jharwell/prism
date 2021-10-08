/**
 * \file specs.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/metrics/specs.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, metrics, specs);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
NS_START(blocks);

cmspecs::name_spec kManipulation = {
  "block_manipulation",
  "blocks/manipulation"
};

NS_END(blocks);

NS_START(gmt);

cmspecs::name_spec kState = {
  "gmt_state",
  "gmt__UUID__/state"
};

cmspecs::name_spec kProgress = {
  "gmt_progress",
  "gmt__UUID__/progress"
};

cmspecs::name_spec kSubtargets = {
  "gmt_subtargets",
  "gmt__UUID__/subtargets"
};

NS_END(gmt);

NS_START(algorithm);

cmspecs::name_spec kLaneAllocation = {
  "algorithm_lane_alloc",
  "algorithm/lane_alloc"
};


NS_END(algorithm);

NS_END(specs, metrics, prism);
