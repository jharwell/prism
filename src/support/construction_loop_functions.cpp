/**
 * \file construction_loop_functions.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/support/construction_loop_functions.hpp"

#include <boost/mpl/for_each.hpp>
#include <cmath>

#include "cosm/arena/config/arena_map_config.hpp"
#include "cosm/controller/operations/applicator.hpp"
#include "cosm/controller/operations/metrics_extract.hpp"
#include "cosm/convergence/convergence_calculator.hpp"
#include "cosm/foraging/metrics/block_transportee_metrics_collector.hpp"
#include "cosm/interactors/applicator.hpp"
#include "cosm/pal/argos/swarm_iterator.hpp"
#include "cosm/pal/config/output_config.hpp"

#include "prism/controller/fcrw_bst_controller.hpp"
#include "prism/controller/perception/builder_perception_subsystem.hpp"
#include "prism/metrics/prism_metrics_manager.hpp"
#include "prism/gmt/ct_manager.hpp"
#include "prism/gmt/spc_gmt.hpp"
#include "prism/support/interactor_status.hpp"
#include "prism/support/robot_arena_interactor.hpp"
#include "prism/support/robot_configurer.hpp"
#include "prism/support/tv/tv_manager.hpp"
#include "prism/gmt/ds/block_anchor_index.hpp"
#include "prism/gmt/repr/vshell.hpp"
#include "prism/support/nearest_ct_calculator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, support);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
NS_START(detail);

using configurer_map_type = rds::type_map<
    rmpl::typelist_wrap_apply<controller::typelist, robot_configurer>::type>;

/**
 * \struct functor_maps_initializer
 * \ingroup support construction detail
 *
 * Convenience class containing initialization for all of the typeid ->
 * std::variant maps for all controller types that are used throughout
 * initialization and simulation.
 */
struct functor_maps_initializer {
  RCPPSW_COLD functor_maps_initializer(configurer_map_type* const cmap,
                                       construction_loop_functions* const lf_in)

      : lf(lf_in), config_map(cmap) {}
  template <typename T>
  RCPPSW_COLD void operator()(const T& controller) const {
    lf->m_interactor_map->emplace(
        typeid(controller),
        robot_arena_interactor<T>(
            lf->arena_map(),
            lf->ct_manager(),
            lf->floor(),
            lf->tv_manager()->dynamics<ctv::dynamics_type::ekENVIRONMENT>(),
            lf->m_metrics_manager.get()));
    lf->m_metrics_map->emplace(
        typeid(controller),
        ccops::metrics_extract<T, pmetrics::prism_metrics_manager>(
            lf->m_metrics_manager.get()));
    config_map->emplace(
        typeid(controller),
        robot_configurer<T>(
            lf->config()->config_get<config::visualization_config>(),
            lf->ct_manager()->targetsro()));
    /*
     * We need to set up the LOS updaters for EVERY possible construction
     * target for EVERY controller type.
     */
    for (size_t i = 0; i < lf->ct_manager()->targetsno().size(); ++i) {
      auto& updater = lf->m_los_updaters[i];
      updater.emplace(typeid(controller),
                      ccops::graph_los_update<T,
                      pgds::connectivity_graph,
                      repr::builder_los>(
                          lf->ct_manager()->targetsno()[i]->spec(),
                          lf->ct_manager()->targetsno()[i]->block_unit_dim()));
    } /* for(i..) */
  }

  /* clang-format off */
  construction_loop_functions * const lf;
  detail::configurer_map_type* const  config_map;
  /* clang-format on */
};

NS_END(detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
construction_loop_functions::construction_loop_functions(void)
    : base_loop_functions(),
      ER_CLIENT_INIT("prism.loop.construction"),
      m_metrics_manager(nullptr),
      m_interactor_map(nullptr),
      m_metrics_map(nullptr) {}

construction_loop_functions::~construction_loop_functions(void) = default;

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/
void construction_loop_functions::init(ticpp::Element& node) {
  ndc_uuid_push();
  ER_INFO("Initializing...");

  shared_init(node);
  private_init();

  ER_INFO("Initialization finished");
  ndc_uuid_pop();
} /* init() */

void construction_loop_functions::shared_init(ticpp::Element& node) {
  base_loop_functions::init(node);

  /* initialize construction */
  const auto* tv = config()->config_get<tv::config::tv_manager_config>();
  auto nests = construction_init(
      config()->config_get<pgconfig::spct_builder_config>(),
      config()->config_get<pgconfig::gmt_config>(),
      &tv->env_dynamics.block_manip_penalty);

  /* initialize arena map, creating nests from construction targets */
  const auto* vconfig = config()->config_get<cavis::config::visualization_config>();
  arena_map_init(vconfig, &nests);
} /* shared_init() */

void construction_loop_functions::private_init(void) {
  /*
   * Need to give spatial metrics collectors the padded arena size in order to
   * avoid boost::assert failures when robots are near the upper edge of the
   * arena map. The arena map pads the size obtained from the XML file after
   * initialization, so we just need to grab it.
   */
  auto padded_size =
      rmath::vector2d(arena_map()->xrsize(), arena_map()->yrsize());
  auto arena = *config()->config_get<caconfig::arena_map_config>();
  const auto* output = config()->config_get<cpconfig::output_config>();
  arena.grid.dims = padded_size;
  m_metrics_manager = std::make_unique<pmetrics::prism_metrics_manager>(
      &output->metrics, &arena.grid, output_root(), ct_manager()->targetsno());

  m_interactor_map = std::make_unique<interactor_map_type>();
  m_metrics_map = std::make_unique<metric_extraction_map_type>();

  /*
   * Each construction target needs its own LOS update map for ALL controller
   * types.
   */
  for (size_t i = 0; i < ct_manager()->targetsno().size(); ++i) {
    m_los_updaters.push_back(los_updater_map_type());
  } /* for(i..) */

  /* only needed for initialization, so not a member */
  auto config_map = detail::configurer_map_type();

  /*
   * Intitialize controller interactions with environment via various
   * functors/type maps for all construction controller types.
   */
  detail::functor_maps_initializer f_initializer(&config_map, this);
  boost::mpl::for_each<controller::typelist>(f_initializer);

  /* configure robots */
  auto cb = [&](auto* controller) {
    ER_ASSERT(config_map.end() != config_map.find(controller->type_index()),
              "Controller '%s' type '%s' not in construction configuration map",
              controller->GetId().c_str(),
              controller->type_index().name());
    auto applicator =
        ccops::applicator<controller::constructing_controller, robot_configurer>(
            controller);
    boost::apply_visitor(applicator, config_map.at(controller->type_index()));
  };

  /*
   * Even though this CAN be done in dynamic order, during initialization ARGoS
   * threads are not set up yet so doing dynamicaly causes a deadlock. Also, it
   * only happens once, so it doesn't really matter if it is slow.
   */
  cpargos::swarm_iterator::controllers<controller::constructing_controller,
                                       cpal::iteration_order::ekSTATIC>(this,
                                                                        cb,
                                                                        cpal::kRobotType);
} /* private_init() */

crepr::config::nests_config construction_loop_functions::construction_init(
    const pgconfig::spct_builder_config* const builder_config,
    const pgconfig::gmt_config* const targets_config,
    const ctv::config::temporal_penalty_config* placement_penalty_config) {
  ER_INFO("Initializing construction targets manager");
  m_ct_manager = std::make_unique<gmt::ct_manager>(
      arena_map(),
      this,
      tv_manager()->dynamics<ctv::dynamics_type::ekENVIRONMENT>());
  return m_ct_manager->initialize(builder_config,
                                  targets_config,
                                  placement_penalty_config);
} /* construction_init() */

/*******************************************************************************
 * ARGoS Hooks
 ******************************************************************************/
void construction_loop_functions::pre_step(void) {
  ndc_uuid_push();
  base_loop_functions::pre_step();
  ndc_uuid_pop();

  /* update structure builders if static builds are enabled */
  m_ct_manager->update(timestep());

  /* Process all robots */
  auto cb = [&](argos::CControllableEntity* robot) {
    ndc_uuid_push();
    robot_pre_step(dynamic_cast<chal::robot&>(robot->GetParent()));
    ndc_uuid_pop();
  };
  cpargos::swarm_iterator::robots<cpal::iteration_order::ekDYNAMIC>(this, cb);
} /* pre_step() */

void construction_loop_functions::post_step(void) {
  ndc_uuid_push();
  base_loop_functions::post_step();
  ndc_uuid_pop();

  /* Process all robots: interact with environment then collect metrics */
  auto cb = [&](argos::CControllableEntity* robot) {
    ndc_uuid_push();
    robot_post_step(dynamic_cast<chal::robot&>(robot->GetParent()));
    ndc_uuid_pop();
  };
  cpargos::swarm_iterator::robots<cpal::iteration_order::ekDYNAMIC>(this, cb);

  ndc_uuid_push();
  /* Update block distribution status */
  const auto* collector =
      m_metrics_manager->get<cfmetrics::block_transportee_metrics_collector>("blocks:"
                                                                         ":transp"
                                                                         "ortee");
  arena_map()->redist_governor()->update(timestep(),
                                         collector->cum_transported(),
                                         false); /* @todo never converged until that stuff is incorporated... */

  /* Collect metrics from loop functions */
  m_metrics_manager->collect_from_ct(ct_manager());

  m_metrics_manager->flush(rmetrics::output_mode::ekTRUNCATE, timestep());
  m_metrics_manager->flush(rmetrics::output_mode::ekCREATE, timestep());

  /* Not a clean way to do this in the metrics collectors... */
  if (m_metrics_manager->flush(rmetrics::output_mode::ekAPPEND, timestep())) {
    tv_manager()->dynamics<ctv::dynamics_type::ekPOPULATION>()->reset_metrics();

    for (auto* target : ct_manager()->targetsno()) {
      target->reset_metrics();
    } /* for(*target..) */
  }

  m_metrics_manager->interval_reset(timestep());

  ndc_uuid_pop();
} /* post_step() */

void construction_loop_functions::destroy(void) {
  if (nullptr != m_metrics_manager) {
    m_metrics_manager->finalize();
  }
} /* destroy() */

void construction_loop_functions::reset(void) {
  ndc_uuid_push();
  base_loop_functions::reset();
  m_metrics_manager->initialize();
  m_ct_manager->reset();
  ndc_uuid_pop();
} /* reset() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void construction_loop_functions::robot_pre_step(chal::robot& robot) {
  auto* controller = static_cast<controller::constructing_controller*>(
      &robot.GetControllableEntity().GetController());

  /*
   * Update robot position, time. This can't be done as part of the robot's
   * control step because we need access to information only available in the
   * loop functions.
   */
  controller->sensing_update(timestep(), arena_map()->grid_resolution());

  /* update robot LOS */
  robot_los_update(controller);
} /* robot_pre_step() */

void construction_loop_functions::robot_post_step(chal::robot& robot) {
  auto* controller = static_cast<controller::constructing_controller*>(
      &robot.GetControllableEntity().GetController());

  /*
   * Watch the robot interact with its environment after physics have been
   * updated and its controller has run.
   */
  auto it = m_interactor_map->find(controller->type_index());
  ER_ASSERT(m_interactor_map->end() != it,
            "Controller '%s' type '%s' not in construction interactor map",
            controller->GetId().c_str(),
            controller->type_index().name());

  auto iapplicator = cinteractors::applicator<controller::constructing_controller,
                                              robot_arena_interactor>(controller,
                                                                      timestep());
  boost::apply_visitor(iapplicator,
                       m_interactor_map->at(controller->type_index()));

  /*
   * Collect metrics from robot, now that it has finished interacting with the
   * environment and no more changes to its state will occur this timestep.
   */
  auto it2 = m_metrics_map->find(controller->type_index());
  ER_ASSERT(m_metrics_map->end() != it2,
            "Controller '%s' type '%s' not in construction metrics map",
            controller->GetId().c_str(),
            controller->type_index().name());
  auto mapplicator =
      ccops::applicator<controller::constructing_controller,
                        ccops::metrics_extract,
                        pmetrics::prism_metrics_manager>(controller);
  if (nullptr != controller->perception()->nearest_ct()) {
    auto visitor = [&](const auto& v) {
      mapplicator(v, controller->perception()->nearest_ct()->id());
    };
    boost::apply_visitor(visitor, m_metrics_map->at(controller->type_index()));
  } else {
    auto visitor = [&](const auto& v) {
      mapplicator(v, rtypes::constants::kNoUUID);
    };
    boost::apply_visitor(visitor, m_metrics_map->at(controller->type_index()));
  }

  controller->block_manip_recorder()->reset();
} /* robot_post_step() */

bool construction_loop_functions::robot_los_update(
    controller::constructing_controller* const c) const {
  auto calculator = nearest_ct_calculator(ct_manager()->targetsno());
  /*
   * Figure out if the robot is current within the 2D bounds of any target, OR
   * if it is JUST outside the 2D bounds of any structure.
   *
   * If it is, then compute and send it a LOS from the gmt, if it is not,
   * then clear out the robot's old LOS so it does not refer to out of date info
   * anymore, and we will get a segfault if it tries to.
   */
  auto* target = calculator.nearest_ct(c);
  auto cell = calculator.nearest_ct_cell(c, target);
  if (nullptr == target || !cell) {
    c->perception()->los(nullptr);
    return false;
  }
  auto nearest_vd = target->anchor_index()->nearest(rds::make_rtree_point(*cell),
                                                    1)[0];
  size_t index = std::distance(ct_manager()->targetsno().begin(),
                               std::find(ct_manager()->targetsno().begin(),
                                         ct_manager()->targetsno().end(),
                                         target));

  auto updater_it = m_los_updaters[index].find(c->type_index());
  ER_ASSERT(m_los_updaters[index].end() != updater_it,
            "Controller '%s' type '%s' not in construction LOS update map",
            c->GetId().c_str(),
            c->type_index().name());
  auto applicator = ccops::applicator<controller::constructing_controller,
                                      ccops::graph_los_update,
                                      pgds::connectivity_graph,
                                      prepr::builder_los>(c);

  auto visitor = std::bind(applicator, std::placeholders::_1, nearest_vd);
  boost::apply_visitor(visitor,
                       m_los_updaters[index].find(c->type_index())->second);
  return true;
} /* robot_los_update() */

using namespace argos; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_LOOP_FUNCTIONS(construction_loop_functions,
                        "construction_loop_functions");

RCPPSW_WARNING_DISABLE_POP()

NS_END(support, prism);
