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

#include "rcppsw/types/type_uuid.hpp"

#include "cosm/metrics/base_metrics_aggregator.hpp"
#include "cosm/metrics/config/metrics_config.hpp"
#include "cosm/ds/config/grid2D_config.hpp"

#include "silicon/silicon.hpp"
#include "silicon/ds/construct_target_vector.hpp"
#include "silicon/controller/controller_fwd.hpp"
#include "silicon/support/tv/tv_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace silicon::structure {
class ct_manager;
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
                             const cdconfig::grid2D_config* gconfig,
                             const std::string& output_root,
                             const ds::construct_target_vectorno& targets);
  ~silicon_metrics_aggregator(void) override = default;

  void collect_from_tv(const support::tv::tv_manager* tvm);
  void collect_from_ct(const structure::ct_manager* manager);
  void collect_from_controller(const controller::constructing_controller* c,
                               const rtypes::type_uuid& structure_id);

 private:
  /**
   * \brief Register collectors that don't need extra arguments that do not
   * pertain to construction targets.
   */
  void register_standard_non_target(const cmconfig::metrics_config* mconfig);

  /**
   * \brief Register collectors that don't need extra arguments that pertain to
   * construction targets.
   */
  void register_standard_target(const cmconfig::metrics_config* mconfig,
                                const sstructure::structure3D* structure);

  /**
   * \brief Register collectors that need extra arguments that pertain to
   * construction targets (# construction lanes).
   */
  void register_with_target_lanes(const cmconfig::metrics_config* mconfig,
                                  const sstructure::structure3D* structure);

  /**
   * \brief Register collectors that need extra arguments that pertain to
   * construction targets (# construction lanes and structure ID).
   */
  void register_with_target_lanes_and_id(const cmconfig::metrics_config* mconfig,
                                         const sstructure::structure3D* structure);

  /**
   * \brief Register collectors that need extra arguments that pertain to
   * construction targets (structure dimenions).
   */
  void register_with_target_dims(const cmconfig::metrics_config* mconfig,
                                 const sstructure::structure3D* structure);
};

NS_END(metrics, silicon);

#endif /* INCLUDE_SILICON_METRICS_SILICON_METRICS_AGGREGATOR_HPP_ */
