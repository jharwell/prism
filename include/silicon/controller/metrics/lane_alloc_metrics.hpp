/**
 * \file lane_alloc_metrics.hpp
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

#ifndef INCLUDE_SILICON_CONTROLLER_METRICS_LANE_ALLOC_METRICS_HPP_
#define INCLUDE_SILICON_CONTROLLER_METRICS_LANE_ALLOC_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"

#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, controller, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class lane_alloc_metrics
 * \ingroup controller metrics
 *
 * \brief Interface defining the metrics to be collected from \ref
 * construction_lane_allocator instances during/about the lane allocation process.
 */
class lane_alloc_metrics : public rmetrics::base_metrics {
 public:
  /**
   * \brief Return the total # of times the construction lane with the specified
   * ID has been allocated.
   */
  virtual size_t alloc_count(size_t id) const = 0;
};

NS_END(metrics, controller, silicon);

#endif /* INCLUDE_SILICON_CONTROLLER_METRICS_LANE_ALLOC_METRICS_HPP_ */
