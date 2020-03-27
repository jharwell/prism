/**
 * \file manipulation_metrics_collector.cpp
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
#include "silicon/metrics/blocks/manipulation_metrics_collector.hpp"

#include "cosm/controller/metrics/manipulation_metrics.hpp"

#include "silicon/metrics/blocks/block_manip_events.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, metrics, blocks);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
manipulation_metrics_collector::manipulation_metrics_collector(
    const std::string& ofname_stem,
    const rtypes::timestep& interval)
    : base_metrics_collector(ofname_stem,
                             interval,
                             rmetrics::output_mode::ekAPPEND) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> manipulation_metrics_collector::csv_header_cols(
    void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
      /* clang-format off */
    "int_avg_arena_pickup_events",
    "int_avg_arena_drop_events",
    "int_avg_arena_pickup_penalty",
    "int_avg_arena_drop_penalty",
      /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void manipulation_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

boost::optional<std::string> manipulation_metrics_collector::csv_line_build(void) {
  if (!(timestep() % interval() == 0)) {
    return boost::none;
  }
  std::string line;

  line += csv_entry_intavg(m_interval.arena_pickup_events);
  line += csv_entry_intavg(m_interval.arena_drop_events);

  line += csv_entry_domavg(m_interval.arena_pickup_penalty,
                           m_interval.arena_pickup_events);
  line += csv_entry_domavg(m_interval.arena_drop_penalty,
                           m_interval.arena_drop_events);

  line += csv_entry_intavg(m_interval.structure_place_events);

  line += csv_entry_domavg(m_interval.structure_place_penalty,
                           m_interval.structure_place_events);

  return boost::make_optional(line);
} /* csv_line_build() */

void manipulation_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const ccmetrics::manipulation_metrics&>(metrics);
  m_interval.arena_pickup_events += m.status(metrics::blocks::block_manip_events::ekARENA_PICKUP);
  m_interval.arena_pickup_penalty += m.penalty(metrics::blocks::block_manip_events::ekARENA_PICKUP).v();

  m_interval.arena_drop_events += m.status(metrics::blocks::block_manip_events::ekARENA_DROP);
  m_interval.arena_drop_penalty += m.penalty(metrics::blocks::block_manip_events::ekARENA_DROP).v();

  m_interval.structure_place_events += m.status(metrics::blocks::block_manip_events::ekSTRUCTURE_PLACE);
  m_interval.structure_place_penalty += m.penalty(metrics::blocks::block_manip_events::ekSTRUCTURE_PLACE).v();
} /* collect() */

void manipulation_metrics_collector::reset_after_interval(void) {
  m_interval.arena_pickup_events = 0;
  m_interval.arena_drop_events = 0;
  m_interval.arena_pickup_penalty = 0;
  m_interval.arena_drop_penalty = 0;

  m_interval.structure_place_events = 0;
  m_interval.structure_place_penalty = 0;
} /* reset_after_interval() */

NS_END(blocks, metrics, silicon);
