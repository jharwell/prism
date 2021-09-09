/**
 * \file ct_state_metrics.hpp
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

#ifndef INCLUDE_PRISM_GMT_METRICS_CT_STATE_METRICS_HPP_
#define INCLUDE_PRISM_GMT_METRICS_CT_STATE_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/gmt/metrics/state_metrics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ct_state_metrics
 * \ingroup gmt metrics
 *
 * \brief Interface defining the metrics to be collected from \ref spc_gmt
 * objects as they are built about current state.
 */
class ct_state_metrics : public state_metrics {};

NS_END(metrics, gmt, prism);

#endif /* INCLUDE_PRISM_GMT_METRICS_CT_STATE_METRICS_HPP_ */
