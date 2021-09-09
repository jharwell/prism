/**
 * \file subtargets_metrics_collector.cpp
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
#include "prism/gmt/metrics/subtargets_metrics_collector.hpp"

#include "prism/gmt/metrics/subtarget_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, gmt, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
subtargets_metrics_collector::subtargets_metrics_collector(
    std::unique_ptr<rmetrics::base_metrics_sink> sink,
    size_t n_subtargets)
    : base_metrics_collector(std::move(sink)),
      m_data(n_subtargets) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void subtargets_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  const auto& m = dynamic_cast<const metrics::subtarget_metrics&>(metrics);
  for (size_t i = 0; i < m_data.interval.size(); ++i) {
    m_data.interval[i].complete_count += m.is_complete();
    m_data.interval[i].placed_count += m.n_interval_placed();
    m_data.interval[i].manifest_size = m.manifest_size();

    m_data.cum[i].complete_count += m.is_complete();
    m_data.cum[i].placed_count += m.n_interval_placed();
    m_data.cum[i].manifest_size = m.manifest_size();
  } /* for(i..) */
} /* collect() */

void subtargets_metrics_collector::reset_after_interval(void) {
  std::fill(m_data.interval.begin(),
            m_data.interval.end(),
            progress_metrics_data{m_data.interval.size()});
} /* reset_after_interval() */

NS_END(metrics, gmt, prism);
