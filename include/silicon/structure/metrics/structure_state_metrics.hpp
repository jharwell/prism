/**
 * \file structure_state_metrics.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_METRICS_STRUCTURE_STATE_METRICS_HPP_
#define INCLUDE_SILICON_STRUCTURE_METRICS_STRUCTURE_STATE_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/math/vector3.hpp"

#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class structure_state_metrics
 * \ingroup structure metrics
 *
 * \brief Interface defining the metrics to be collected from \ref structure3D
 * as it is built about block counts, etc.
 */
class structure_state_metrics : public virtual rmetrics::base_metrics {
 public:
  /**
   * \brief Return the list of cells within the structure that contain blocks.
   */
  virtual std::vector<rmath::vector3z> occupied_cells(void) const = 0;
};

NS_END(metrics, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_METRICS_STRUCTURE_STATE_METRICS_HPP_ */
