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
#include "rcppsw/metrics/file_sink_registerer.hpp"
#include "rcppsw/metrics/register_with_sink.hpp"
#include "rcppsw/metrics/register_using_config.hpp"
#include "rcppsw/mpl/identity.hpp"

#include "cosm/convergence/convergence_calculator.hpp"
#include "cosm/metrics/specs.hpp"

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
#include "prism/metrics/specs.hpp"

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
      fs_output_manager(mconfig, output_root) {
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
  register_standard(mconfig);

  for (auto* target : targets) {
    register_with_ct(mconfig, target);
    register_with_ct_and_lanes(mconfig, target);
    register_with_ct_and_lanes_and_id(mconfig, target);
    register_with_ct_and_dims(mconfig, target);
  } /* for(*target..) */

  /* setup metric collection for all collector groups in all sink groups */
  initialize();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void prism_metrics_manager::collect_from_tv(
    const support::tv::tv_manager* const tvm) {
  collect(cmspecs::tv::kEnvironment.scoped(),
          *tvm->dynamics<ctv::dynamics_type::ekENVIRONMENT>());

  if (nullptr != tvm->dynamics<ctv::dynamics_type::ekPOPULATION>()) {
    collect(cmspecs::tv::kPopulation.scoped(),
            *tvm->dynamics<ctv::dynamics_type::ekPOPULATION>());
  }
} /* collect_from_tv() */

void prism_metrics_manager::collect_from_ct(
    const gmt::ct_manager* manager) {
  for (const auto* target : manager->targetsro()) {
    collect(pmspecs::gmt::kState.scoped(target->id()), *target);
    collect(pmspecs::gmt::kProgress.scoped(target->id()), *target);
    for (auto* st : target->subtargets()) {
      collect(pmspecs::gmt::kSubtargets.scoped(target->id()), *st);
    } /* for(*t..) */
  } /* for(*target..) */
} /* collect_from_ct() */

template <typename TController>
void prism_metrics_manager::collect_from_controller(
    const TController* c,
    const rtypes::type_uuid& ct_id) {
  fs_output_manager::collect_from_controller(c);

  if (rtypes::constants::kNoUUID != ct_id) {
    collect(pmspecs::algorithm::kLaneAllocation.scoped(ct_id),
            *c->fsm()->lane_allocator());
  }
  collect(pmspecs::blocks::kManipulation.scoped(),
          *c->block_manip_recorder());
  collect(cmspecs::blocks::kAcqLocs2D.scoped(), *c);
  collect(cmspecs::blocks::kAcqExploreLocs2D.scoped(), *c);
  collect(cmspecs::blocks::kAcqVectorLocs2D.scoped(), *c);
  collect(cmspecs::blocks::kAcqCounts.scoped(), *c);
  collect(cmspecs::spatial::kInterferenceCounts.scoped(), *c->fsm());
} /* collect_from_controller() */

void prism_metrics_manager::register_standard(
    const rmconfig::metrics_config* mconfig) {
  using sink_list = rmpl::typelist<
      rmpl::identity<blocks::manipulation_metrics_csv_sink>,
      rmpl::identity<support::tv::metrics::env_dynamics_metrics_csv_sink>
    >;
  rmetrics::creatable_collector_set creatable_set = {
    {
      typeid(blocks::manipulation_metrics_collector),
      pmspecs::blocks::kManipulation.xml(),
      pmspecs::blocks::kManipulation.scoped(),
      rmetrics::output_mode::ekAPPEND
    },
    {
      typeid(support::tv::metrics::env_dynamics_metrics_collector),
      cmspecs::tv::kEnvironment.xml(),
      cmspecs::tv::kEnvironment.scoped(),
      rmetrics::output_mode::ekAPPEND
    }
  };

  rmetrics::register_with_sink<pmetrics::prism_metrics_manager,
                               rmetrics::file_sink_registerer> csv(this,
                                                                   creatable_set);
  rmetrics::register_using_config<decltype(csv),
                                  rmconfig::file_sink_config> registerer(
                                      std::move(csv),
                                      &mconfig->csv);

  boost::mpl::for_each<sink_list>(registerer);
} /* register_standard() */

void prism_metrics_manager::register_with_ct(
    const rmconfig::metrics_config* mconfig,
    const pgmt::spc_gmt* ct) {
  using sink_list = rmpl::typelist<
      rmpl::identity<gmt::metrics::ct_progress_metrics_csv_sink>
    >;
  rmetrics::creatable_collector_set creatable_set = {
    {
      typeid(gmt::metrics::ct_progress_metrics_collector),
      pmspecs::gmt::kProgress.xml(ct->id()),
      pmspecs::gmt::kProgress.scoped(ct->id()),
      rmetrics::output_mode::ekAPPEND
    },
  };
  rmetrics::register_with_sink<pmetrics::prism_metrics_manager,
                               rmetrics::file_sink_registerer> csv(this,
                                                                   creatable_set);
  rmetrics::register_using_config<decltype(csv),
                                  rmconfig::file_sink_config> registerer(
                                      std::move(csv),
                                      &mconfig->csv);

  boost::mpl::for_each<sink_list>(registerer);
} /* register_with_ct() */

void prism_metrics_manager::register_with_ct_and_lanes(
    const rmconfig::metrics_config* mconfig,
    const pgmt::spc_gmt* ct) {
  using sink_list = rmpl::typelist<
      rmpl::identity<pgmetrics::subtargets_metrics_csv_sink>
    >;

  rmetrics::creatable_collector_set creatable_set = {
    {
      typeid(gmt::metrics::subtargets_metrics_collector),
      pmspecs::gmt::kSubtargets.xml(ct->id()),
      pmspecs::gmt::kSubtargets.scoped(ct->id()),
      rmetrics::output_mode::ekAPPEND
    },
  };
  auto extra_args = std::make_tuple(ct->subtargets().size());
  rmetrics::register_with_sink<pmetrics::prism_metrics_manager,
                               rmetrics::file_sink_registerer,
                               decltype(extra_args)> csv(this,
                                                         creatable_set,
                                                         extra_args);
  rmetrics::register_using_config<decltype(csv),
                                  rmconfig::file_sink_config> registerer(
                                      std::move(csv),
                                      &mconfig->csv);

  boost::mpl::for_each<sink_list>(registerer);
} /* register_with_ct_and_lanes() */

void prism_metrics_manager::register_with_ct_and_lanes_and_id(
    const rmconfig::metrics_config* mconfig,
    const pgmt::spc_gmt* ct) {
  using sink_list = rmpl::typelist<
        rmpl::identity<plametrics::lane_alloc_metrics_csv_sink>
    >;

  rmetrics::creatable_collector_set creatable_set = {
    {
      typeid(plametrics::lane_alloc_metrics_collector),
      pmspecs::algorithm::kLaneAllocation.xml(ct->id()),
      pmspecs::algorithm::kLaneAllocation.scoped(ct->id()),
      rmetrics::output_mode::ekAPPEND
    }
  };
  auto extra_args = std::make_tuple(ct->id(),
                                    ct->subtargets().size());

  rmetrics::register_with_sink<pmetrics::prism_metrics_manager,
                               rmetrics::file_sink_registerer,
                               decltype(extra_args)> csv(this,
                                                         creatable_set,
                                                         extra_args);
  rmetrics::register_using_config<decltype(csv),
                                  rmconfig::file_sink_config> registerer(
                                      std::move(csv),
                                      &mconfig->csv);
  boost::mpl::for_each<sink_list>(registerer);
} /* register_with_ct_and_lanes_and_id() */

void prism_metrics_manager::register_with_ct_and_dims(
    const rmconfig::metrics_config* mconfig,
    const pgmt::spc_gmt* ct) {

  using sink_list = rmpl::typelist<
    rmpl::identity<pgmetrics::ct_state_metrics_csv_sink>
    >;

  rmetrics::creatable_collector_set creatable_set = {
    {
      typeid(gmt::metrics::ct_state_metrics_collector),
      pmspecs::gmt::kState.xml(ct->id()),
      pmspecs::gmt::kState.scoped(ct->id()),
      rmetrics::output_mode::ekCREATE | rmetrics::output_mode::ekTRUNCATE
    },
  };

  auto extra_args = std::make_tuple(rmath::vector3z{ct->xdsize(),
                    ct->ydsize(),
                    ct->zdsize()});

  rmetrics::register_with_sink<pmetrics::prism_metrics_manager,
                               rmetrics::file_sink_registerer,
                               decltype(extra_args)> csv(this,
                                                         creatable_set,
                                                         extra_args);
  rmetrics::register_using_config<decltype(csv),
                                  rmconfig::file_sink_config> registerer(
                                      std::move(csv),
                                      &mconfig->csv);

  boost::mpl::for_each<sink_list>(registerer);
} /* register_with_ct_and_dims() */

/*******************************************************************************
 * Template Instantiations
 ******************************************************************************/
template void prism_metrics_manager::collect_from_controller(
    const controller::fcrw_bst_controller*,
    const rtypes::type_uuid&);

NS_END(metrics, prism);
