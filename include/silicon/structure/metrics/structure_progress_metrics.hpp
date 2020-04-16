/**
 * \file structure_progress_metrics.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_METRICS_STRUCTURE_PROGRESS_METRICS_HPP_
#define INCLUDE_SILICON_STRUCTURE_METRICS_STRUCTURE_PROGRESS_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/structure/metrics/progress_metrics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class structure_progress_metrics
 * \ingroup structure metrics
 *
 * \brief Interface defining the metrics to be collected from \ref structure3D
 * objects as they are built.
 */
class structure_progress_metrics : public progress_metrics {};

NS_END(metrics, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_METRICS_STRUCTURE_PROGRESS_METRICS_HPP_ */
