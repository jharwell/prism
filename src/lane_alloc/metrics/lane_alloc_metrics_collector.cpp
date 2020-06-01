/**
 * \file lane_alloc_metrics_collector.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/lane_alloc/metrics/lane_alloc_metrics_collector.hpp"

#include "silicon/lane_alloc/metrics/lane_alloc_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, lane_alloc, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
lane_alloc_metrics_collector::lane_alloc_metrics_collector(
    const std::string& ofname_stem,
    const rtypes::timestep& interval,
    const rtypes::type_uuid& target_id,
    size_t n_lanes)
    : base_metrics_collector(ofname_stem,
                             interval,
                             rmetrics::output_mode::ekAPPEND),
      mc_target_id(target_id),
      m_interval(n_lanes),
      m_cum(n_lanes) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> lane_alloc_metrics_collector::csv_header_cols(
    void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>();

  for (size_t i = 0; i < m_interval.size(); ++i) {
    cols.push_back("int_avg_lane" + std::to_string(i) + "_alloc_count");
    cols.push_back("cum_avg_lane" + std::to_string(i) + "_alloc_count");
  } /* for(i..) */

  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void lane_alloc_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

boost::optional<std::string> lane_alloc_metrics_collector::csv_line_build(void) {
  if (!(timestep() % interval() == 0)) {
    return boost::none;
  }
  std::string line;

  for (size_t i = 0; i < m_interval.size(); ++i) {
    line += csv_entry_intavg(m_interval[i].alloc_count);
    line += csv_entry_tsavg(m_cum[i].alloc_count);
  } /* for(i..) */

  return boost::make_optional(line);
} /* csv_line_build() */

void lane_alloc_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const metrics::lane_alloc_metrics&>(metrics);
  for (size_t i = 0; i < m_interval.size(); ++i) {
    m_interval[i].alloc_count += m.alloc_count(mc_target_id, i);
    m_cum[i].alloc_count += m.alloc_count(mc_target_id, i);
  } /* for(i..) */
} /* collect() */

void lane_alloc_metrics_collector::reset_after_interval(void) {
  std::fill(m_interval.begin(), m_interval.end(), stats{});
} /* reset_after_interval() */

NS_END(metrics, lane_alloc, silicon);
