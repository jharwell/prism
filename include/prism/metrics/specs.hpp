/**
 * \file specs.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include "cosm/metrics/name_spec.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, metrics, specs);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

NS_START(blocks);

extern cmspecs::name_spec kManipulation;

NS_END(blocks);

NS_START(gmt);

extern cmspecs::name_spec kState;
extern cmspecs::name_spec kProgress;
extern cmspecs::name_spec kSubtargets;

NS_END(gmt);

NS_START(algorithm);

extern cmspecs::name_spec kLaneAllocation;

NS_END(algorithm);

NS_END(specs, metrics, prism);
