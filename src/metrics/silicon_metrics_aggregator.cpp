/**
 * \file silicon_metrics_aggregator.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/metrics/silicon_metrics_aggregator.hpp"

#include <boost/mpl/for_each.hpp>

#include "rcppsw/mpl/typelist.hpp"

#include "cosm/convergence/convergence_calculator.hpp"
#include "cosm/metrics/collector_registerer.hpp"

#include "silicon/controller/constructing_controller.hpp"
#include "silicon/metrics/blocks/manipulation_metrics_collector.hpp"
#include "silicon/support/base_loop_functions.hpp"
#include "silicon/support/tv/metrics/env_dynamics_metrics.hpp"
#include "silicon/support/tv/metrics/env_dynamics_metrics_collector.hpp"
#include "silicon/support/tv/tv_manager.hpp"
#include "silicon/structure/metrics/structure_progress_metrics_collector.hpp"
#include "silicon/structure/metrics/structure_state_metrics_collector.hpp"
#include "silicon/structure/metrics/subtargets_metrics_collector.hpp"
#include "silicon/structure/subtarget.hpp"
#include "silicon/structure/structure3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, metrics, detail);

using collector_typelist = rmpl::typelist<
    rmpl::identity<blocks::manipulation_metrics_collector>,
    rmpl::identity<support::tv::metrics::env_dynamics_metrics_collector>>;

NS_END(detail);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
silicon_metrics_aggregator::silicon_metrics_aggregator(
    const cmconfig::metrics_config* const mconfig,
    const cdconfig::grid_config* const gconfig,
    const std::string& output_root)
    : ER_CLIENT_INIT("silicon.metrics.aggregator"),
      base_metrics_aggregator(mconfig, gconfig, output_root) {
  cmetrics::collector_registerer::creatable_set creatable_set = {
      {typeid(blocks::manipulation_metrics_collector),
       "block_manipulation",
       "blocks::manipulation",
       rmetrics::output_mode::ekAPPEND},
      {typeid(support::tv::metrics::env_dynamics_metrics_collector),
       "tv_environment",
       "tv::environment",
       rmetrics::output_mode::ekAPPEND},
      {typeid(structure::metrics::structure_state_metrics_collector),
       "structure_state",
       "structure::state",
       rmetrics::output_mode::ekCREATE | rmetrics::output_mode::ekTRUNCATE},
      {typeid(structure::metrics::structure_progress_metrics_collector),
       "structure_progress",
       "structure::progress",
       rmetrics::output_mode::ekAPPEND},
      {typeid(structure::metrics::subtargets_metrics_collector),
       "structure_subtargets",
       "structure::subtargets",
       rmetrics::output_mode::ekAPPEND}
  };

  cmetrics::collector_registerer registerer(
      mconfig, gconfig, creatable_set, this);
  boost::mpl::for_each<detail::collector_typelist>(registerer);
  reset_all();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void silicon_metrics_aggregator::collect_from_loop(
    const support::base_loop_functions* const loop) {
  collect("tv::environment",
          *loop->tv_manager()->dynamics<ctv::dynamics_type::ekENVIRONMENT>());

  if (loop->tv_manager()->dynamics<ctv::dynamics_type::ekPOPULATION>()) {
    collect("tv::population",
            *loop->tv_manager()->dynamics<ctv::dynamics_type::ekPOPULATION>());
  }
} /* collect_from_loop() */

void silicon_metrics_aggregator::collect_from_structure(
    const structure::structure3D* structure) {
  collect("structure::state", *structure);
  collect("structure::progress", *structure);

  for (auto *st : structure->subtargets()) {
    collect("structure::subtargets", *st);
  } /* for(*t..) */
} /* collect_from_structure() */

NS_END(metrics, silicon);
