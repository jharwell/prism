/**
 * \file structure_progress_metrics_collector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "silicon/structure/metrics/structure_progress_metrics_collector.hpp"

#include "silicon/structure/metrics/structure_progress_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, structure, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
structure_progress_metrics_collector::structure_progress_metrics_collector(
    const std::string& ofname_stem,
    const rtypes::timestep& interval)
    : base_metrics_collector(ofname_stem,
                             interval,
                             rmetrics::output_mode::ekAPPEND) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
structure_progress_metrics_collector::csv_header_cols(void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_avg_complete_count",
    "cum_avg_complete_count",
    "int_avg_placed_count",
    "cum_avg_placed_count",
    "int_avg_manifest_size",
    "cum_avg_manifest_size",
    /* clang-format on */
  };

  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void structure_progress_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

boost::optional<std::string>
structure_progress_metrics_collector::csv_line_build(void) {
  if (!(timestep() % interval() == 0)) {
    return boost::none;
  }
  std::string line;

  /* complete counts */
  line += csv_entry_intavg(m_interval.complete_count);
  line += csv_entry_tsavg(m_cum.complete_count);

  /* placed counts */
  line += csv_entry_intavg(m_interval.placed_count);
  line += csv_entry_tsavg(m_cum.placed_count);

  /* manifest sizes */
  line += rcppsw::to_string(m_interval.manifest_size) + separator();
  line += rcppsw::to_string(m_cum.manifest_size) + separator();

  return boost::make_optional(line);
} /* csv_line_build() */

void structure_progress_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const metrics::structure_progress_metrics&>(metrics);
  m_interval.complete_count += m.is_complete();
  m_interval.placed_count += m.n_interval_placed();
  m_interval.manifest_size = m.manifest_size();

  m_cum.complete_count += m.is_complete();
  m_cum.placed_count += m.n_total_placed();
  m_cum.manifest_size = m.manifest_size();
} /* collect() */

void structure_progress_metrics_collector::reset_after_interval(void) {
  m_interval.complete_count = 0;
  m_interval.placed_count = 0;
} /* reset_after_interval() */

NS_END(metrics, structure, silicon);
