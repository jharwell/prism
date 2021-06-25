/**
 * \file env_dynamics_metrics_collector.cpp
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
#include "silicon/support/tv/metrics/env_dynamics_metrics_collector.hpp"

#include "silicon/support/tv/metrics/env_dynamics_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, support, tv, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
env_dynamics_metrics_collector::env_dynamics_metrics_collector(
    std::unique_ptr<rmetrics::base_metrics_sink> sink)
    : base_metrics_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void env_dynamics_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  const auto& m = dynamic_cast<const env_dynamics_metrics&>(metrics);
  m_data.avg_motion_throttle = m.avg_motion_throttle();
  m_data.arena_block_manip_penalty = m.arena_block_manip_penalty();
  m_data.structure_block_manip_penalty = m.ct_block_manip_penalty();
} /* collect() */

NS_END(metrics, tv, support, silicon);
