/**
 * \file lane_alloc_metrics.hpp
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
#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, lane_alloc, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class lane_alloc_metrics
 * \ingroup lane_alloc metrics
 *
 * \brief Interface defining the metrics to be collected from \ref allocator
 * instances during/about the lane allocation process.
 */
class lane_alloc_metrics : public rmetrics::base_metrics {
 public:
  /**
   * \brief Return the total # of times the construction lane with the specified
   * ID has been allocated on the specified target structure.
   */
  virtual size_t alloc_count(const rtypes::type_uuid& target,
                             size_t id) const = 0;
};

NS_END(metrics, lane_alloc, prism);

