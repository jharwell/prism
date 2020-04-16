/**
 * \file silicon_metrics_aggregator.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_SILICON_METRICS_SILICON_METRICS_AGGREGATOR_HPP_
#define INCLUDE_SILICON_METRICS_SILICON_METRICS_AGGREGATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/metrics/base_metrics_aggregator.hpp"
#include "cosm/metrics/config/metrics_config.hpp"

#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace silicon::support {
class base_loop_functions;
}
namespace silicon::structure {
class structure3D;
} /* namespace silicon::structure */

NS_START(silicon, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class silicon_metrics_aggregator
 * \ingroup metrics
 *
 * \brief Extends the \ref cmetrics::base_metrics_aggregator for the SILICON
 * project.
 */
class silicon_metrics_aggregator
    : public rer::client<silicon_metrics_aggregator>,
      public cmetrics::base_metrics_aggregator {
 public:
  silicon_metrics_aggregator(const cmconfig::metrics_config* mconfig,
                             const cdconfig::grid_config* const gconfig,
                             const std::string& output_root);
  ~silicon_metrics_aggregator(void) override = default;

  void collect_from_loop(const support::base_loop_functions* loop);
  void collect_from_structure(const structure::structure3D* structure);
};

NS_END(metrics, silicon);

#endif /* INCLUDE_SILICON_METRICS_SILICON_METRICS_AGGREGATOR_HPP_ */
