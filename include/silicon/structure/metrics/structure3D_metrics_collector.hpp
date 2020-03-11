/**
 * \file goal_acq_locs_metrics_collector.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_METRICS_STRUCTURE3D_METRICS_COLLECTOR_HPP_
#define INCLUDE_SILICON_STRUCTURE_METRICS_STRUCTURE3D_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <list>

#include "rcppsw/metrics/spatial/grid3D_metrics_collector.hpp"
#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, structure, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class structure3D_metrics_collector
 * \ingroup structure metrics
 *
 * \brief Collector for construction progress as reported directly from \ref
 * structure3D.
 *
 * Metrics MUST be collected serially; concurrent updates to the gathered stats
 * are not supported.
 */
class structure3D_metrics_collector final :
    public rmetrics::spatial::grid3D_metrics_collector<rmetrics::spatial::cell_avg> {
 public:
  /**
   * \param ofname The output file name.
   * \param interval Collection interval.
   * \param dims Dimensions of the structure.
   * \param mode The selected output mode.
   */
  structure3D_metrics_collector(const std::string& ofname,
                                  const rtypes::timestep& interval,
                                  const rmath::vector3u& dims,
                                  const rmetrics::output_mode& mode) :
      grid3D_metrics_collector(ofname, interval, dims, mode) {}

  void collect(const rmetrics::base_metrics& metrics) override;
};

NS_END(metrics, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_METRICS_STRUCTURE3D_METRICS_COLLECTOR_HPP_ */
