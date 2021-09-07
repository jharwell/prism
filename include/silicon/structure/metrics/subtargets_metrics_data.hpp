/**
 * \file subtargets_metrics_data.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_SILICON_STRUCTURE_METRICS_SUBTARGETS_METRICS_DATA_HPP_
#define INCLUDE_SILICON_STRUCTURE_METRICS_SUBTARGETS_METRICS_DATA_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/metrics/base_metrics_data.hpp"

#include "silicon/structure/metrics/progress_metrics_data.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
struct subtargets_metrics_data : public rmetrics::base_metrics_data  {
  explicit subtargets_metrics_data(size_t n_subtargets):
      interval(n_subtargets),
      cum(n_subtargets) {}

  std::vector<progress_metrics_data> interval;
  std::vector<progress_metrics_data> cum;
};

NS_END(metrics, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_METRICS_SUBTARGETS_METRICS_DATA_HPP_ */
