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
#include "silicon/controller/metrics/lane_alloc_metrics_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, metrics);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
silicon_metrics_aggregator::silicon_metrics_aggregator(
    const cmconfig::metrics_config* const mconfig,
    const cdconfig::grid_config* const gconfig,
    const std::string& output_root,
    const ds::construct_target_vectorno& targets)
    : ER_CLIENT_INIT("silicon.metrics.aggregator"),
      base_metrics_aggregator(mconfig, output_root) {
  /* register collectors from base class  */
  auto dims2D = rmath::dvec2zvec(gconfig->upper,
                                 gconfig->resolution.v());
  size_t max_height = 0;
  for (auto *target : targets) {
    max_height = std::max(target->zsize(), max_height);
  } /* for(*target..) */

  /* robots can be on top of structures, so give a little padding in Z */
  auto dims3D = rmath::vector3z(dims2D.x(),
                                dims2D.y(),
                                max_height + 2);

  register_with_arena_dims2D(mconfig, dims2D);
  register_with_arena_dims3D(mconfig, dims3D);

  /* register SILICON collectors */
  register_standard_non_target(mconfig);
  for (auto *target : targets) {
    register_standard_target(mconfig, target);
    register_with_target_lanes(mconfig, target);
    register_with_target_dims(mconfig, target);
  } /* for(*target..) */
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
  collect("structure" + rcppsw::to_string(structure->id()) + "::state",
          *structure);
  collect("structure" + rcppsw::to_string(structure->id()) + "::progress",
          *structure);

  for (auto *st : structure->subtargets()) {
    collect("structure" + rcppsw::to_string(structure->id()) + "::subtargets",
            *st);
  } /* for(*t..) */
} /* collect_from_structure() */

void silicon_metrics_aggregator::register_standard_non_target(
    const cmconfig::metrics_config* mconfig) {
  using collector_typelist = rmpl::typelist<
    rmpl::identity<blocks::manipulation_metrics_collector>,
    rmpl::identity<support::tv::metrics::env_dynamics_metrics_collector>
    >;
  cmetrics::collector_registerer<>::creatable_set creatable_set = {
    {typeid(blocks::manipulation_metrics_collector),
     "block_manipulation",
     "blocks::manipulation",
     rmetrics::output_mode::ekAPPEND},
    {typeid(support::tv::metrics::env_dynamics_metrics_collector),
     "tv_environment",
     "tv::environment",
     rmetrics::output_mode::ekAPPEND}
  };
  cmetrics::collector_registerer<> registerer(mconfig,
                                              creatable_set,
                                              this);
  boost::mpl::for_each<collector_typelist>(registerer);
} /* register_standard_non_target() */

void silicon_metrics_aggregator::register_standard_target(
    const cmconfig::metrics_config* mconfig,
    const sstructure::structure3D* structure) {
  using collector_typelist = rmpl::typelist<
    rmpl::identity<structure::metrics::structure_progress_metrics_collector>
    >;
  cmetrics::collector_registerer<>::creatable_set creatable_set = {
    {typeid(structure::metrics::structure_progress_metrics_collector),
     "structure" + rcppsw::to_string(structure->id()) + "_progress",
     "structure" + rcppsw::to_string(structure->id()) + "::progress",
     rmetrics::output_mode::ekAPPEND},
  };
  cmetrics::collector_registerer<> registerer(mconfig,
                                              creatable_set,
                                              this);
  boost::mpl::for_each<collector_typelist>(registerer);
} /* register_standard_target() */

void silicon_metrics_aggregator::register_with_target_lanes(
    const cmconfig::metrics_config* mconfig,
    const sstructure::structure3D* structure) {
  using extra_args_type = std::tuple<size_t>;
  using collector_typelist = rmpl::typelist<
    rmpl::identity<structure::metrics::subtargets_metrics_collector>,
    rmpl::identity<controller::metrics::lane_alloc_metrics_collector>
    >;

  cmetrics::collector_registerer<extra_args_type>::creatable_set creatable_set = {
    {typeid(structure::metrics::subtargets_metrics_collector),
     "structure" + rcppsw::to_string(structure->id()) + "_subtargets",
     "structure" + rcppsw::to_string(structure->id()) + "::subtargets",
     rmetrics::output_mode::ekAPPEND},
    {typeid(controller::metrics::lane_alloc_metrics_collector),
     "structure" + rcppsw::to_string(structure->id()) + "_lane_alloc",
     "structure" + rcppsw::to_string(structure->id()) + "::lane_alloc",
     rmetrics::output_mode::ekAPPEND}
  };
  auto extra_args = std::make_tuple(structure->subtargets().size());
  cmetrics::collector_registerer<extra_args_type> registerer(mconfig,
                                                             creatable_set,
                                                             this,
                                                             extra_args);
  boost::mpl::for_each<collector_typelist>(registerer);
} /* register_with_target_lanes() */

void silicon_metrics_aggregator::register_with_target_dims(
    const cmconfig::metrics_config* mconfig,
    const sstructure::structure3D* structure) {
  using extra_args_type = std::tuple<rmath::vector3z>;
  using collector_typelist = rmpl::typelist<
    rmpl::identity<structure::metrics::structure_state_metrics_collector>
    >;
  cmetrics::collector_registerer<extra_args_type>::creatable_set creatable_set = {
    {typeid(structure::metrics::structure_state_metrics_collector),
     "structure" + rcppsw::to_string(structure->id()) + "_state",
     "structure" + rcppsw::to_string(structure->id()) + "::state",
     rmetrics::output_mode::ekCREATE | rmetrics::output_mode::ekTRUNCATE},
  };

  auto extra_args = std::make_tuple(structure->dims());
  cmetrics::collector_registerer<extra_args_type> registerer(mconfig,
                                                             creatable_set,
                                                             this,
                                                             extra_args);
  boost::mpl::for_each<collector_typelist>(registerer);
} /* register_with_target_dims() */

NS_END(metrics, silicon);
