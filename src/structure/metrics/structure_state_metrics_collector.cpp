/**
 * \file structure_state_metrics_collector.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/structure/metrics/structure_state_metrics_collector.hpp"

#include "cosm/repr/base_block3D.hpp"

#include "silicon/structure/metrics/structure_state_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, structure, metrics);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void structure_state_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  const auto& m = dynamic_cast<const structure_state_metrics&>(metrics);
  inc_total_count(m.occupied_cells().size());

  for (auto& cell : m.occupied_cells()) {
    inc_cell_count(cell);
  } /* for(*b..) */
} /* collect() */

NS_END(metrics, structure, silicon);
