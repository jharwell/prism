/**
 * \file ct_state_metrics_csv_sink.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include "rcppsw/ds/metrics/grid3D_metrics_csv_sink.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, metrics);
class ct_state_metrics_collector;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class ct_state_metrics_csv_sink :
    public rdmetrics::grid3D_metrics_csv_sink<rdmetrics::cell_avg> {
 public:
  using collector_type = ct_state_metrics_collector;
  using rdmetrics::grid3D_metrics_csv_sink<rdmetrics::cell_avg>::grid3D_metrics_csv_sink;
};

NS_END(metrics, gmt, prism);

