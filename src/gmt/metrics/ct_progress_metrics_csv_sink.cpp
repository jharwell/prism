/**
 * \file ct_progress_metrics_csv_sink.cpp
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
#include "prism/gmt/metrics/ct_progress_metrics_csv_sink.hpp"

#include "prism/gmt/metrics/ct_progress_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, gmt, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
ct_progress_metrics_csv_sink::ct_progress_metrics_csv_sink(
    fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
ct_progress_metrics_csv_sink::csv_header_cols(
    const rmetrics::base_data*) const {
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

boost::optional<std::string>
ct_progress_metrics_csv_sink::csv_line_build(const rmetrics::base_data* data,
                                          const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }
  auto* d = static_cast<const ct_progress_metrics_data*>(data);
  std::string line;

  /* complete counts */
  line += csv_entry_intavg(d->interval.complete_count);
  line += csv_entry_tsavg(d->cum.complete_count, t);

  /* placed counts */
  line += csv_entry_intavg(d->interval.placed_count);
  line += csv_entry_tsavg(d->cum.placed_count, t);

  /* manifest sizes */
  line += rcppsw::to_string(d->interval.manifest_size) + separator();
  line += rcppsw::to_string(d->cum.manifest_size);

  return boost::make_optional(line);
} /* csv_line_build() */

NS_END(metrics, gmt, prism);
