/**
 * \file lane_allocs_metrics_collector.hpp
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

#ifndef INCLUDE_SILICON_CONTROLLER_METRICS_LANE_ALLOC_METRICS_COLLECTOR_HPP_
#define INCLUDE_SILICON_CONTROLLER_METRICS_LANE_ALLOC_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <list>
#include <vector>

#include "rcppsw/metrics/base_metrics_collector.hpp"
#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, controller, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class lane_alloc_metrics_collector
 * \ingroup controller metrics
 *
 * \brief Collector for \ref lane_alloc_metrics within a single \ref
 * structure3D.
 *
 * Metrics CAN be collected in parallel from robots; concurrent updates to the
 * gathered stats are supported. Metrics are written out at the specified
 * collection interval.
 */
class lane_alloc_metrics_collector final : public rmetrics::base_metrics_collector {
 public:
  /**
   * \param ofname_stem The output file name stem.
   * \param interval Collection interval.
   * \param n_lanes # construct lanes in target.
   */
  lane_alloc_metrics_collector(const std::string& ofname_stem,
                               const rtypes::timestep& interval,
                               size_t n_lanes);

  void reset(void) override;
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;

 private:
  struct stats {
    size_t alloc_count{0};
  };

  std::list<std::string> csv_header_cols(void) const override;

  boost::optional<std::string> csv_line_build(void) override;

  /* clang-format off */
  std::vector<stats> m_interval{};
  std::vector<stats> m_cum{};
  /* clang-format on */
};

NS_END(metrics, controller, silicon);

#endif /* INCLUDE_SILICON_CONTROLLER_METRICS_LANE_ALLOC_METRICS_COLLECTOR_HPP_ */
