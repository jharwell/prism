/**
 * \file subtargets_metrics_collector.hpp
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

#ifndef INCLUDE_PRISM_GMT_METRICS_SUBTARGETS_METRICS_COLLECTOR_HPP_
#define INCLUDE_PRISM_GMT_METRICS_SUBTARGETS_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/metrics/base_metrics_collector.hpp"

#include "prism/gmt/metrics/subtargets_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, gmt, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class subtargets_metrics_collector
 * \ingroup gmt metrics
 *
 * \brief Collector for \ref subtarget_metrics for ALL subtargets within a \ref
 * spc_gmt.
 *
 * Metrics MUST be collected serially; concurrent updates to the gathered stats
 * are not supported.
 */
class subtargets_metrics_collector final : public rmetrics::base_metrics_collector {
 public:
 /**
   * \param sink The metrics sink to use.
   *
   * \param n_subtargets The # of subtargets the structure will be broken down
   *                     into during construction.
   */
  subtargets_metrics_collector(std::unique_ptr<rmetrics::base_metrics_sink> sink,
                               size_t n_subtargets);

  /* base_metrics_collector overrides */
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;
  const rmetrics::base_metrics_data* data(void) const override { return &m_data; }

 private:
  /* clang-format off */
  subtargets_metrics_data m_data;
  /* clang-format on */
};

NS_END(metrics, gmt, prism);

#endif /* INCLUDE_PRISM_GMT_METRICS_SUBTARGETS_METRICS_COLLECTOR_HPP_ */
