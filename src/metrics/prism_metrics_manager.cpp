/**
 * \file prism_metrics_manager.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/metrics/prism_metrics_manager.hpp"

#include <boost/mpl/for_each.hpp>

#include "rcppsw/mpl/typelist.hpp"
#include "rcppsw/utils/maskable_enum.hpp"
#include "rcppsw/metrics/collector_registerer.hpp"

#include "cosm/convergence/convergence_calculator.hpp"

#include "prism/controller/fcrw_bst_controller.hpp"
#include "prism/fsm/fcrw_bst_fsm.hpp"
#include "prism/lane_alloc/metrics/lane_alloc_metrics_collector.hpp"
#include "prism/lane_alloc/metrics/lane_alloc_metrics_csv_sink.hpp"
#include "prism/metrics/blocks/manipulation_metrics_collector.hpp"
#include "prism/metrics/blocks/manipulation_metrics_csv_sink.hpp"
#include "prism/gmt/ct_manager.hpp"
#include "prism/gmt/metrics/ct_progress_metrics_collector.hpp"
#include "prism/gmt/metrics/ct_progress_metrics_csv_sink.hpp"
#include "prism/gmt/metrics/ct_state_metrics_collector.hpp"
#include "prism/gmt/metrics/ct_state_metrics_csv_sink.hpp"
#include "prism/gmt/metrics/subtargets_metrics_collector.hpp"
#include "prism/gmt/metrics/subtargets_metrics_csv_sink.hpp"
#include "prism/gmt/spc_gmt.hpp"
#include "prism/gmt/subtarget.hpp"
#include "prism/support/tv/metrics/env_dynamics_metrics_collector.hpp"
#include "prism/support/tv/metrics/env_dynamics_metrics_csv_sink.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, metrics);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
prism_metrics_manager::prism_metrics_manager(
    const rmconfig::metrics_config* const mconfig,
    const cdconfig::grid2D_config* const gconfig,
    const fs::path& output_root,
    const ds::ct_vectorno& targets)
    : ER_CLIENT_INIT("prism.metrics.manager"),
      cosm_metrics_manager(mconfig, output_root) {
  /* register collectors from base class  */
  auto dims2D = rmath::dvec2zvec(gconfig->dims, gconfig->resolution.v());
  size_t max_height = 0;
  for (auto* target : targets) {
    max_height = std::max(target->zdsize(), max_height);
  } /* for(*target..) */

  /* robots can be on top of structures, so give a little padding in Z */
  auto dims3D = rmath::vector3z(dims2D.x(), dims2D.y(), max_height + 2);

  register_with_arena_dims2D(mconfig, dims2D);
  register_with_arena_dims3D(mconfig, dims3D);

  /* register PRISM collectors */
  register_standard_non_target(mconfig);
  for (auto* target : targets) {
    register_standard_target(mconfig, target);
    register_with_target_lanes(mconfig, target);
    register_with_target_lanes_and_id(mconfig, target);
    register_with_target_dims(mconfig, target);
  } /* for(*target..) */

  /* setup metric collection for all collector groups in all sink groups */
  initialize();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void prism_metrics_manager::collect_from_tv(
    const support::tv::tv_manager* const tvm) {
  collect("tv::environment", *tvm->dynamics<ctv::dynamics_type::ekENVIRONMENT>());

  if (nullptr != tvm->dynamics<ctv::dynamics_type::ekPOPULATION>()) {
    collect("tv::population", *tvm->dynamics<ctv::dynamics_type::ekPOPULATION>());
  }
} /* collect_from_tv() */

void prism_metrics_manager::collect_from_ct(
    const gmt::ct_manager* manager) {
  for (const auto* target : manager->targetsro()) {
    collect("structure" + rcppsw::to_string(target->id()) + "::state", *target);
    collect("structure" + rcppsw::to_string(target->id()) + "::progress",
            *target);
    for (auto* st : target->subtargets()) {
      collect("structure" + rcppsw::to_string(target->id()) + "::subtargets",
              *st);
    } /* for(*t..) */
  } /* for(*target..) */
} /* collect_from_ct() */

template <typename TController>
void prism_metrics_manager::collect_from_controller(
    const TController* c,
    const rtypes::type_uuid& structure_id) {
  cosm_metrics_manager::collect_from_controller(c);

  if (rtypes::constants::kNoUUID != structure_id) {
    collect("structure" + rcppsw::to_string(structure_id) + "::lane_alloc",
            *c->fsm()->lane_allocator());
  }
  collect("blocks::manipulation", *c->block_manip_recorder());
  collect("blocks::acq_locs2D", *c);
  collect("blocks::acq_explore_locs2D", *c);
  collect("blocks::acq_vector_locs2D", *c);
  collect("blocks::acq_counts", *c);
  collect("fsm::interference_counts", *c->fsm());
} /* collect_from_controller() */

void prism_metrics_manager::register_standard_non_target(
    const rmconfig::metrics_config* mconfig) {
  using sink_list = rmpl::typelist<
      rmpl::identity<blocks::manipulation_metrics_csv_sink>,
      rmpl::identity<support::tv::metrics::env_dynamics_metrics_csv_sink>
    >;
  rmetrics::creatable_collector_set creatable_set = {
    { typeid(blocks::manipulation_metrics_collector),
      "block_manipulation",
      "blocks::manipulation",
      rmetrics::output_mode::ekAPPEND },
    { typeid(support::tv::metrics::env_dynamics_metrics_collector),
      "tv_environment",
      "tv::environment",
      rmetrics::output_mode::ekAPPEND }
  };
  rmetrics::register_with_csv_sink csv(&mconfig->csv,
                                       creatable_set,
                                       this);
  rmetrics::collector_registerer<decltype(csv)> registerer(std::move(csv));
  boost::mpl::for_each<sink_list>(registerer);
} /* register_standard_non_target() */

void prism_metrics_manager::register_standard_target(
    const rmconfig::metrics_config* mconfig,
    const pgmt::spc_gmt* structure) {
  using sink_list = rmpl::typelist<
      rmpl::identity<gmt::metrics::ct_progress_metrics_csv_sink>
    >;
  rmetrics::creatable_collector_set creatable_set = {
    { typeid(gmt::metrics::ct_progress_metrics_collector),
      "structure" + rcppsw::to_string(structure->id()) + "_progress",
      "structure" + rcppsw::to_string(structure->id()) + "::progress",
      rmetrics::output_mode::ekAPPEND },
  };
  rmetrics::register_with_csv_sink csv(&mconfig->csv,
                                       creatable_set,
                                       this);
  rmetrics::collector_registerer<decltype(csv)> registerer(std::move(csv));
  boost::mpl::for_each<sink_list>(registerer);
} /* register_standard_target() */

void prism_metrics_manager::register_with_target_lanes(
    const rmconfig::metrics_config* mconfig,
    const pgmt::spc_gmt* structure) {
  using sink_list = rmpl::typelist<
      rmpl::identity<pgmetrics::subtargets_metrics_csv_sink>
    >;

  rmetrics::creatable_collector_set creatable_set = {
    { typeid(gmt::metrics::subtargets_metrics_collector),
      "structure" + rcppsw::to_string(structure->id()) + "_subtargets",
      "structure" + rcppsw::to_string(structure->id()) + "::subtargets",
      rmetrics::output_mode::ekAPPEND },
  };
  auto extra_args = std::make_tuple(structure->subtargets().size());
  rmetrics::register_with_csv_sink<decltype(extra_args)> csv(&mconfig->csv,
                                                             creatable_set,
                                                             this,
                                                             extra_args);
  rmetrics::collector_registerer<decltype(csv)> registerer(std::move(csv));
  boost::mpl::for_each<sink_list>(registerer);
} /* register_with_target_lanes() */

void prism_metrics_manager::register_with_target_lanes_and_id(
    const rmconfig::metrics_config* mconfig,
    const pgmt::spc_gmt* structure) {
  using sink_list = rmpl::typelist<
        rmpl::identity<plametrics::lane_alloc_metrics_csv_sink>
    >;

  rmetrics::creatable_collector_set creatable_set = {
    { typeid(plametrics::lane_alloc_metrics_collector),
      "structure" + rcppsw::to_string(structure->id()) + "_lane_alloc",
      "structure" + rcppsw::to_string(structure->id()) + "::lane_alloc",
      rmetrics::output_mode::ekAPPEND }
  };
  auto extra_args = std::make_tuple(structure->id(),
                                    structure->subtargets().size());
  rmetrics::register_with_csv_sink<decltype(extra_args)> csv(&mconfig->csv,
                                                            creatable_set,
                                                             this,
                                                             extra_args);
  rmetrics::collector_registerer<decltype(csv)> registerer(std::move(csv));
  boost::mpl::for_each<sink_list>(registerer);
} /* register_with_target_lanes_and_id() */

void prism_metrics_manager::register_with_target_dims(
    const rmconfig::metrics_config* mconfig,
    const pgmt::spc_gmt* structure) {

  using sink_list = rmpl::typelist<
    rmpl::identity<pgmetrics::ct_state_metrics_csv_sink>
    >;

  rmetrics::creatable_collector_set creatable_set = {
    { typeid(gmt::metrics::ct_state_metrics_collector),
      "structure" + rcppsw::to_string(structure->id()) + "_state",
      "structure" + rcppsw::to_string(structure->id()) + "::state",
      rmetrics::output_mode::ekCREATE | rmetrics::output_mode::ekTRUNCATE },
  };

  auto extra_args = std::make_tuple(rmath::vector3z{structure->xdsize(),
                    structure->ydsize(),
                    structure->zdsize()});
  rmetrics::register_with_csv_sink<decltype(extra_args)> csv(&mconfig->csv,
                                                             creatable_set,
                                                             this,
                                                             extra_args);
  rmetrics::collector_registerer<decltype(csv)> registerer(std::move(csv));
  boost::mpl::for_each<sink_list>(registerer);
} /* register_with_target_dims() */

/*******************************************************************************
 * Template Instantiations
 ******************************************************************************/
template void prism_metrics_manager::collect_from_controller(
    const controller::fcrw_bst_controller*,
    const rtypes::type_uuid&);

NS_END(metrics, prism);
