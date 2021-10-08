/**
 * \file construction_loop_functions.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <vector>

#include "rcppsw/ds/type_map.hpp"
#include "rcppsw/mpl/typelist.hpp"

#include "cosm/controller/operations/metrics_extract.hpp"
#include "cosm/controller/operations/graph_los_update.hpp"
#include "cosm/hal/robot.hpp"

#include "prism/controller/controller_fwd.hpp"
#include "prism/support/base_loop_functions.hpp"
#include "prism/gmt/ds/connectivity_graph.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace prism::metrics {
class prism_metrics_manager;
} /* namespace prism::metrics */

namespace prism::repr {
class builder_los;
} /* namespace prism::repr */

namespace prism::gmt {
class spc_gmt;
class ct_manager;
} /* namespace prism::gmt */

NS_START(prism, support);

template <typename ControllerType>
class robot_arena_interactor;

namespace detail {
struct functor_maps_initializer;
} /* namespace detail */

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class construction_loop_functions
 * \ingroup support construction
 *
 * \brief Contains the simulation support functions for construction,
 * such as:
 *
 * - Metric collection from robots
 * - Robot arena interactions
 */
class construction_loop_functions
    : public base_loop_functions,
      public rer::client<construction_loop_functions> {
 public:
  construction_loop_functions(void) RCPPSW_COLD;
  ~construction_loop_functions(void) override RCPPSW_COLD;

  /* swarm manager overrides */
  void init(ticpp::Element& node) override RCPPSW_COLD;
  void pre_step(void) override;
  void post_step(void) override;
  void reset(void) override RCPPSW_COLD;
  void destroy(void) override RCPPSW_COLD;

 protected:
  /**
   * \brief Initialize construction support to be shared with derived classes:
   *
   * - Construction metric collection
   */
  void shared_init(ticpp::Element& node) RCPPSW_COLD;

  const pgmt::ct_manager* ct_manager(void) const {
    return m_ct_manager.get();
  }

  pgmt::ct_manager* ct_manager(void) { return m_ct_manager.get(); }

 private:
  using interactor_map_type =
      rds::type_map<rmpl::typelist_wrap_apply<controller::typelist,
                                              robot_arena_interactor>::type>;

  using los_updater_map_type =
      rds::type_map<rmpl::typelist_wrap_apply<controller::typelist,
                                              ccops::graph_los_update,
                                              pgds::connectivity_graph,
                                              repr::builder_los>::type>;

  using metric_extraction_map_type = rds::type_map<
      rmpl::typelist_wrap_apply<controller::typelist,
                                ccops::metrics_extract,
                                metrics::prism_metrics_manager>::type>;

  using los_updater_vector = std::vector<los_updater_map_type>;

  /**
   * \brief These are friend classes because they are basically just pieces of
   * the loop functions pulled out for increased clarity/modularity, and are not
   * meant to be used in other contexts.
   *
   * Doing things this way rather than passing 8 parameters to the functors
   * seemed much cleaner.
   */
  friend detail::functor_maps_initializer;

  /**
   * \brief Initialize construction support not shared with derived classes:
   *
   * - Robot interactions with arena
   * - Various maps mapping controller types to metric collection, controller
   *   initialization, and arena interaction maps (reflection basically).
   */
  void private_init(void) RCPPSW_COLD;

  /**
   * \brief Process a single robot on a timestep, before running its controller:
   *
   * - Set its new position, time from ARGoS and send it its LOS.
   *
   * \note These operations are done in parallel for all robots (lock free).
   */
  void robot_pre_step(chal::robot& robot);

  /**
   * \brief Process a single robot on a timestep, after running its controller.
   *
   * - Have it interact with the environment.
   * - Collect metrics from it.
   *
   * \note These operations are done in parallel for all robots (with mutual
   *       exclusion as needed).
   */
  void robot_post_step(chal::robot& robot);

  /**
   * \brief Update the LOS for a robot currently on the structure somewhere and
   * NOT in the 2D arena (if they are currently somewhere in the 2D arena, then
   * nothing is done).
   *
   * In this project, robots do not have intelligence when foraging for objects,
   * hence no LOS, because that is not the focus of it--construction is.
   *
   * \return \c TRUE if the  LOS for a robot was updated (i.e. the robot WAS
   * on a structure somewhere), and \c FALSE otherwise.
   */
  bool robot_los_update(controller::constructing_controller* c) const;

  gmt::spc_gmt*
  robot_target(const controller::constructing_controller* c) const;


  /**
   * \brief Initialize the \ref gmt::ct_manager.
   *
   * \param builder_config Parsed builder parameters.
   * \param target_config Parsed \ref gmt::spc_gmt parameters (one per
   *                      structure).
   */
  crepr::config::nests_config construction_init(
      const pgconfig::spct_builder_config* builder_config,
      const pgconfig::gmt_config* targets_config,
      const ctv::config::temporal_penalty_config* placement_penalty_config);

  /* clang-format off */
  std::unique_ptr<gmt::ct_manager>                m_ct_manager{nullptr};
  std::unique_ptr<metrics::prism_metrics_manager> m_metrics_manager;
  std::unique_ptr<interactor_map_type>            m_interactor_map;
  std::unique_ptr<metric_extraction_map_type>     m_metrics_map;
  los_updater_vector                              m_los_updaters{};
  /* clang-format on */
};

NS_END(support, prism);

