/**
 * \file ct_progress_metrics_collector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "prism/gmt/metrics/ct_progress_metrics_collector.hpp"

#include "prism/gmt/metrics/progress_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, gmt, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
ct_progress_metrics_collector::ct_progress_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void ct_progress_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  const auto& m =
      dynamic_cast<const metrics::progress_metrics&>(metrics);
  m_data.interval.complete_count += m.is_complete();
  m_data.interval.placed_count += m.n_interval_placed();
  m_data.interval.manifest_size = m.manifest_size();

  m_data.cum.complete_count += m.is_complete();
  m_data.cum.placed_count += m.n_interval_placed();
  m_data.cum.manifest_size = m.manifest_size();
} /* collect() */

void ct_progress_metrics_collector::reset_after_interval(void) {
  m_data.interval.complete_count = 0;
  m_data.interval.placed_count = 0;
} /* reset_after_interval() */

NS_END(metrics, gmt, prism);
