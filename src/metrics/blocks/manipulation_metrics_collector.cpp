/**
 * \file manipulation_metrics_collector.cpp
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
#include "prism/metrics/blocks/manipulation_metrics_collector.hpp"

#include "cosm/controller/metrics/manipulation_metrics.hpp"

#include "prism/metrics/blocks/block_manip_events.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, metrics, blocks);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
manipulation_metrics_collector::manipulation_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void manipulation_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  const auto& m = dynamic_cast<const ccmetrics::manipulation_metrics&>(metrics);
  m_data.interval.arena_pickup_events +=
      m.status(metrics::blocks::block_manip_events::ekARENA_PICKUP);
  m_data.interval.arena_pickup_penalty +=
      m.penalty(metrics::blocks::block_manip_events::ekARENA_PICKUP).v();

  m_data.interval.structure_place_events +=
      m.status(metrics::blocks::block_manip_events::ekSPCT_PLACE);
  m_data.interval.structure_place_penalty +=
      m.penalty(metrics::blocks::block_manip_events::ekSPCT_PLACE).v();

  m_data.cum.arena_pickup_events +=
      m.status(metrics::blocks::block_manip_events::ekARENA_PICKUP);
  m_data.cum.arena_pickup_penalty +=
      m.penalty(metrics::blocks::block_manip_events::ekARENA_PICKUP).v();

  m_data.cum.structure_place_events +=
      m.status(metrics::blocks::block_manip_events::ekSPCT_PLACE);
  m_data.cum.structure_place_penalty +=
      m.penalty(metrics::blocks::block_manip_events::ekSPCT_PLACE).v();
} /* collect() */

void manipulation_metrics_collector::reset_after_interval(void) {
  m_data.interval.arena_pickup_events = 0;
  m_data.interval.arena_pickup_penalty = 0;

  m_data.interval.structure_place_events = 0;
  m_data.interval.structure_place_penalty = 0;
} /* reset_after_interval() */

NS_END(blocks, metrics, prism);
