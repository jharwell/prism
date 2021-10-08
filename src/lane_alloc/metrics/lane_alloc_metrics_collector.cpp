/**
 * \file lane_alloc_metrics_collector.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/lane_alloc/metrics/lane_alloc_metrics_collector.hpp"

#include "prism/lane_alloc/metrics/lane_alloc_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, lane_alloc, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
lane_alloc_metrics_collector::lane_alloc_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink,
    const rtypes::type_uuid& target_id,
    size_t n_lanes)
    : base_collector(std::move(sink)),
      mc_target_id(target_id),
      m_data(n_lanes) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void lane_alloc_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  const auto& m = dynamic_cast<const metrics::lane_alloc_metrics&>(metrics);
  for (size_t i = 0; i < m_data.interval.size(); ++i) {
    m_data.interval[i].alloc_count += m.alloc_count(mc_target_id, i);
    m_data.cum[i].alloc_count += m.alloc_count(mc_target_id, i);
  } /* for(i..) */
} /* collect() */

void lane_alloc_metrics_collector::reset_after_interval(void) {
  for (auto& stats : m_data.interval) {
    stats.alloc_count = 0;
  } /* for(stats&..) */
} /* reset_after_interval() */

NS_END(metrics, lane_alloc, prism);
