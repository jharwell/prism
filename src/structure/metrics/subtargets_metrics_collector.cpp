/**
 * \file subtargets_metrics_collector.cpp
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
#include "silicon/structure/metrics/subtargets_metrics_collector.hpp"

#include "silicon/structure/metrics/subtarget_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, structure, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
subtargets_metrics_collector::subtargets_metrics_collector(
    const std::string& ofname_stem,
    const rtypes::timestep& interval,
    size_t n_subtargets)
    : base_metrics_collector(ofname_stem,
                             interval,
                             rmetrics::output_mode::ekAPPEND),
      m_interval(n_subtargets),
      m_cum(n_subtargets) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> subtargets_metrics_collector::csv_header_cols(void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>();
  for (size_t i = 0; i < m_interval.size(); ++i) {
    cols.push_back("int_avg_subtarget" + std::to_string(i) + "_placed_count");
    cols.push_back("cum_avg_subtarget" + std::to_string(i) + "_placed_count");
    cols.push_back("subtarget" + std::to_string(i) + "_size_in_blocks");
  } /* for(i..) */

  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void subtargets_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

boost::optional<std::string> subtargets_metrics_collector::csv_line_build(void) {
  if (!(timestep() % interval() == 0)) {
    return boost::none;
  }
  std::string line;

  for (size_t i = 0; i < m_interval.size(); ++i) {
    line += csv_entry_intavg(m_interval[i].n_placed_count);
    line += csv_entry_tsavg(m_cum[i].n_placed_count);
    line += rcppsw::to_string(m_cum[i].n_total_count) + separator();
  } /* for(i..) */

  return boost::make_optional(line);
} /* csv_line_build() */

void subtargets_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const metrics::subtarget_metrics&>(metrics);
  for (size_t i = 0; i < m_interval.size(); ++i) {
    m_interval[i].n_placed_count += m.n_placed_blocks();
    m_cum[i].n_placed_count += m.n_placed_blocks();
    m_cum[i].n_total_count = m.n_total_blocks();
  } /* for(i..) */
} /* collect() */

void subtargets_metrics_collector::reset_after_interval(void) {
  std::fill(m_interval.begin(), m_interval.end(), stats{});
} /* reset_after_interval() */

NS_END(metrics, structure, silicon);
