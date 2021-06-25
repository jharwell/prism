/**
 * \file ct_progress_metrics_csv_sink.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_METRICS_CT_PROGRESS_METRICS_CSV_SINK_HPP_
#define INCLUDE_SILICON_STRUCTURE_METRICS_CT_PROGRESS_METRICS_CSV_SINK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <string>

#include "rcppsw/metrics/csv_sink.hpp"

#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, structure, metrics);
class ct_progress_metrics_collector;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ct_progress_metrics_csv_sink
 * \ingroup support metrics tv
 *
 * \brief Sink for \ref ct_progress_metrics and \ref
 * ct_progress_metrics_collector to output metrics to .csv.
 */
class ct_progress_metrics_csv_sink final : public rmetrics::csv_sink {
 public:
  using collector_type = ct_progress_metrics_collector;

  /**
   * \brief \see rmetrics::csv_sink.
   */
  ct_progress_metrics_csv_sink(fs::path fpath_no_ext,
                                const rmetrics::output_mode& mode,
                                const rtypes::timestep& interval);

  /* csv_sink overrides */
  std::list<std::string> csv_header_cols(
      const rmetrics::base_metrics_data* data) const override;

  boost::optional<std::string> csv_line_build(
      const rmetrics::base_metrics_data* data,
      const rtypes::timestep& t) override;
};

NS_END(metrics, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_METRICS_CT_PROGRESS_METRICS_CSV_SINK_HPP_ */
