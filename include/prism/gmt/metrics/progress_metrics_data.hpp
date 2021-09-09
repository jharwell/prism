/**
 * \file progress_metrics_data.hpp
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

#ifndef INCLUDE_PRISM_GMT_METRICS_PROGRESS_METRICS_DATA_HPP_
#define INCLUDE_PRISM_GMT_METRICS_PROGRESS_METRICS_DATA_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct progress_metrics_data
 * \ingroup gmt metrics
 *
 * \brief Container for holding collected statistics of \ref
 * progress_metrics. Does not need to be atomic, because these are collected
 * in non-current contexts.
 */
struct progress_metrics_data {
  size_t placed_count{0};
  size_t manifest_size{0};
  size_t complete_count{0};
};

NS_END(metrics, gmt, prism);

#endif /* INCLUDE_PRISM_GMT_METRICS_PROGRESS_METRICS_DATA_HPP_ */
