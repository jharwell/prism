/**
 * \file ct_state_metrics_collector.hpp
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

#include "rcppsw/ds/metrics/grid3D_metrics_collector.hpp"
#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, gmt, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ct_state_metrics_collector
 * \ingroup gmt metrics
 *
 * \brief Collector for construction progress as reported directly from \ref
 * spc_gmt.
 *
 * Metrics MUST be collected serially; concurrent updates to the gathered stats
 * are not supported.
 */
class ct_state_metrics_collector final : public rdmetrics::grid3D_metrics_collector {
 public:
  /**
   * \param sink The metrics sink to use.
   * \param dims Dimensions of structure.
   */
  ct_state_metrics_collector(
      std::unique_ptr<rmetrics::base_sink> sink,
      const rmath::vector3z& dims)
      : grid3D_metrics_collector(std::move(sink), dims) {}

  void collect(const rmetrics::base_metrics& metrics) override;
};

NS_END(metrics, gmt, prism);

