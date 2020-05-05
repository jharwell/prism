/**
 * \file construction_acq_goal.hpp
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

#ifndef INCLUDE_SILICON_FSM_CONSTRUCTION_ACQ_GOAL_HPP_
#define INCLUDE_SILICON_FSM_CONSTRUCTION_ACQ_GOAL_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/metrics/goal_acq_metrics.hpp"

#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, fsm);

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * \brief Representats the different types of objects/locations robots can
 * acquire as part of the construction process.
 */
enum class construction_acq_goal {
  ekNONE = -1,
  ekBLOCK,
  ekSTRUCTURE,
};

/*******************************************************************************
 * Operators
 ******************************************************************************/
bool operator==(const csmetrics::goal_acq_metrics::goal_type& goal1,
                const construction_acq_goal& goal2) RCSW_PURE;

bool operator==(const construction_acq_goal& goal1,
                const csmetrics::goal_acq_metrics::goal_type& goal2) RCSW_PURE;

bool operator!=(const csmetrics::goal_acq_metrics::goal_type& goal1,
                const construction_acq_goal& goal2) RCSW_PURE;

bool operator!=(const construction_acq_goal& goal1,
                const csmetrics::goal_acq_metrics::goal_type& goal2) RCSW_PURE;

csmetrics::goal_acq_metrics::goal_type to_goal_type(
    const construction_acq_goal& goal);

NS_END(fsm, silicon);

#endif /* INCLUDE_SILICON_FSM_CONSTRUCTION_ACQ_GOAL_HPP_ */
