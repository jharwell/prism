/**
 * \file ct_progress_metrics_collector.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_METRICS_CT_PROGRESS_METRICS_COLLECTOR_HPP_
#define INCLUDE_SILICON_STRUCTURE_METRICS_CT_PROGRESS_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics_collector.hpp"

#include "silicon/structure/metrics/ct_progress_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, structure, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ct_progress_metrics_collector
 * \ingroup structure metrics
 *
 * \brief Collector for \ref ct_progress_metrics.
 *
 * Metrics MUST be collected serially; concurrent updates to the gathered stats
 * are not supported.
 */
class ct_progress_metrics_collector final : public rmetrics::base_metrics_collector {
 public:
  /**
   * \param sink The metrics sink to use.
   */
  explicit ct_progress_metrics_collector(
      std::unique_ptr<rmetrics::base_metrics_sink> sink);

  /* base_metrics_collector overrides */
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;
  const rmetrics::base_metrics_data* data(void) const override { return &m_data; }

 private:
  /* clang-format off */
  ct_progress_metrics_data m_data{};
  /* clang-format on */
};

NS_END(metrics, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_METRICS_CT_PROGRESS_METRICS_COLLECTOR_HPP_ */
