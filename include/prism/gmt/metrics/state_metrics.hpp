/**
 * \file state_metrics.hpp
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
#include <vector>

#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/math/vector3.hpp"

#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class state_metrics
 * \ingroup gmt metrics
 *
 * \brief Interface defining the common metrics to be collected from \ref
 * spc_gmt, \ref subtarget objects as they are built regarding current
 * state, as opposed to progress.
 */
class state_metrics : public virtual rmetrics::base_metrics {
 public:
  /**
   * \brief Return the list of cells within the structure that contain blocks.
   */
  virtual std::vector<rmath::vector3z> occupied_cells(void) const = 0;
};

NS_END(metrics, gmt, prism);

