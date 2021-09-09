/**
 * \file env_dynamics_metrics_collector.hpp
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

#ifndef INCLUDE_PRISM_SUPPORT_TV_METRICS_ENV_DYNAMICS_METRICS_COLLECTOR_HPP_
#define INCLUDE_PRISM_SUPPORT_TV_METRICS_ENV_DYNAMICS_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/metrics/base_metrics_collector.hpp"

#include "prism/support/tv/metrics/env_dynamics_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, support, tv, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class env_dynamics_metrics_collector
 * \ingroup support metrics tv
 *
 * \brief Collector for \ref env_dynamics_metrics.
 *
 * Metrics CANNOT be collected in parallel; concurrent updates to the gathered
 * stats are not supported. Metrics are written out every timestep.
 */
class env_dynamics_metrics_collector final : public rmetrics::base_metrics_collector {
 public:
  /**
   * \param sink The metrics sink to use.
   */
  explicit env_dynamics_metrics_collector(
      std::unique_ptr<rmetrics::base_metrics_sink> sink);

  /* base_metrics_collector overrides */
  void collect(const rmetrics::base_metrics& metrics) override;
  const rmetrics::base_metrics_data* data(void) const override { return &m_data; }


 private:
  /* clang-format off */
  env_dynamics_metrics_data m_data{};
  /* clang-format on */
};

NS_END(metrics, tv, support, prism);

#endif /* INCLUDE_PRISM_SUPPORT_TV_METRICS_ENV_DYNAMICS_METRICS_COLLECTOR_HPP_ */
