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
    const std::string& ofname_stem)
    : base_metrics_collector(ofname_stem,
                             rtypes::timestep(1),
                             rmetrics::output_mode::ekAPPEND) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> env_dynamics_metrics_collector::csv_header_cols(
    void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
      /* clang-format off */
      "swarm_motion_throttle",
      "arena_block_manip_penalty",
      "ct_block_manip_penalty"
      /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

boost::optional<std::string> env_dynamics_metrics_collector::csv_line_build(void) {
  std::string line;
  line += rcppsw::to_string(m_avg_motion_throttle) + separator();
  line += rcppsw::to_string(m_arena_block_manip_penalty) + separator();
  line += rcppsw::to_string(m_structure_block_manip_penalty);
  return boost::make_optional(line);
} /* csv_line_build() */

void env_dynamics_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const env_dynamics_metrics&>(metrics);
  m_avg_motion_throttle = m.avg_motion_throttle();
  m_arena_block_manip_penalty = m.arena_block_manip_penalty();
  m_structure_block_manip_penalty = m.ct_block_manip_penalty();
} /* collect() */

NS_END(metrics, tv, support, silicon);
