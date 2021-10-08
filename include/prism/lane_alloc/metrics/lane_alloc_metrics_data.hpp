/**
 * \file lane_alloc_metrics_data.hpp
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
#include <atomic>
#include <vector>


#include "rcppsw/metrics/base_data.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, lane_alloc, metrics, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct lane_alloc_metrics_data
 * \ingroup lane_alloc metrics detail
 *
 * \brief Container for holding collected statistics of \ref
 * lane_alloc_metrics. Must be atomic so counts are valid in parallel metric
 * collection contexts.
 */
struct lane_alloc_metrics_data {
  std::atomic_size_t alloc_count{0};
};

NS_END(detail);

struct lane_alloc_metrics_data : public rmetrics::base_data {
  explicit lane_alloc_metrics_data(size_t n_lanes)
      : interval(n_lanes),
        cum(n_lanes) {}

  std::vector<detail::lane_alloc_metrics_data> interval;
  std::vector<detail::lane_alloc_metrics_data> cum;
};

NS_END(metrics, lane_alloc, prism);

