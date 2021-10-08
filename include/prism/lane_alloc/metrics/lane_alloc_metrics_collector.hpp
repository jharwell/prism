/**
 * \file lane_allocs_metrics_collector.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <list>
#include <vector>

#include "rcppsw/metrics/base_collector.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "prism/prism.hpp"
#include "prism/lane_alloc/metrics/lane_alloc_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, lane_alloc, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class lane_alloc_metrics_collector
 * \ingroup lane_alloc metrics
 *
 * \brief Collector for \ref lane_alloc_metrics within a single \ref
 * spc_gmt.
 *
 * Metrics CAN be collected in parallel from robots; concurrent updates to the
 * gathered stats are supported. Metrics are written out at the specified
 * collection interval.
 */
class lane_alloc_metrics_collector final : public rmetrics::base_collector {
 public:
  /**
   * \param sink The metrics sink to use.
   *
   * \param target_id ID of the target structure from which lane allocation
   *                  metrics will be gathered.
   * \param n_lanes # of lanes on the target structure.
   */
  lane_alloc_metrics_collector(
      std::unique_ptr<rmetrics::base_sink> sink,
      const rtypes::type_uuid& target_id,
      size_t n_lanes);

  /* base_collector overrides */
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;
  const rmetrics::base_data* data(void) const override { return &m_data; }

 private:
  /* clang-format off */
  const rtypes::type_uuid mc_target_id;

  lane_alloc_metrics_data m_data;
  /* clang-format on */
};

NS_END(metrics, lane_alloc, prism);

