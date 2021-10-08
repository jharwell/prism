/**
 * \file manipulation_metrics_csv_sink.cpp
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
#include "prism/metrics/blocks/manipulation_metrics_csv_sink.hpp"

#include "prism/metrics/blocks/manipulation_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, metrics, blocks);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
manipulation_metrics_csv_sink::manipulation_metrics_csv_sink(
    fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
manipulation_metrics_csv_sink::csv_header_cols(
    const rmetrics::base_data*) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_avg_arena_pickup_events",
    "int_avg_arena_pickup_penalty",
    "int_avg_ct_placement_events",
    "int_avg_ct_placement_penalty",
    "cum_avg_arena_pickup_events",
    "cum_avg_arena_pickup_penalty",
    "cum_avg_ct_placement_events",
    "cum_avg_ct_placement_penalty",
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

boost::optional<std::string>
manipulation_metrics_csv_sink::csv_line_build(
    const rmetrics::base_data* data,
    const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }

  auto* d = static_cast<const manipulation_metrics_data*>(data);

  std::string line;

  line += csv_entry_intavg(d->interval.arena_pickup_events);
  line += csv_entry_domavg(d->interval.arena_pickup_penalty,
                           d->interval.arena_pickup_events);

  line += csv_entry_intavg(d->interval.structure_place_events);
  line += csv_entry_domavg(d->interval.structure_place_penalty,
                           d->interval.structure_place_events);

  line += csv_entry_tsavg(d->cum.arena_pickup_events, t);
  line += csv_entry_domavg(d->cum.arena_pickup_penalty,
                           d->cum.arena_pickup_events);

  line += csv_entry_tsavg(d->cum.structure_place_events, t);
  line += csv_entry_domavg(d->cum.structure_place_penalty,
                           d->cum.structure_place_events,
                           true);

  return boost::make_optional(line);
} /* csv_line_build() */

NS_END(blocks, metrics, prism);
