/**
 * \file fcrw_bst_controller.cpp
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
#include "prism/controller/fcrw_bst_controller.hpp"

#include "cosm/fsm/supervisor_fsm.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"

#include "prism/controller/config/constructing_controller_repository.hpp"
#include "prism/fsm/fcrw_bst_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
fcrw_bst_controller::fcrw_bst_controller(void)
    : ER_CLIENT_INIT("prism.controller.fcrw_bst"), m_fsm() {}

fcrw_bst_controller::~fcrw_bst_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void fcrw_bst_controller::init(ticpp::Element& node) {
  constructing_controller::init(node);
  ndc_uuid_push();
  ER_INFO("Initializing...");

  config::constructing_controller_repository repo;
  repo.parse_all(node);

  const auto* allocator_config = repo.config_get<placonfig::lane_alloc_config>();
  pfsm::fsm_params params = {
    {
      saa(),
      inta_tracker(),
      nullptr,
    },
    perception()
  };

  m_fsm = std::make_unique<fsm::fcrw_bst_fsm>(allocator_config,
                                              &params,
                                              rng());

  /* Set FCRW_BST FSM supervision */
  supervisor()->supervisee_update(m_fsm.get());
  ER_INFO("Initialization finished");
  ndc_uuid_pop();
} /* init() */

void fcrw_bst_controller::reset(void) {
  constructing_controller::reset();
  if (nullptr != m_fsm) {
    m_fsm->init();
  }
} /* reset() */

void fcrw_bst_controller::control_step(void) {
  mdc_ts_update();
  ndc_uuid_push();

  ER_ASSERT(!(nullptr != block() &&
              rtypes::constants::kNoUUID == block()->md()->robot_id()),
            "Carried block%d has robot id=%d",
            block()->id().v(),
            block()->md()->robot_id().v());

  /*
   * Reset steering force tracking from the previous time step. Cannot be done
   * on the same timestep because visualizations are drawn AFTER the control
   * step runs.
   */
  saa()->steer_force2D().tracking_reset();

  /* Reset lane allocation metrics so we don't double count allocations */
  m_fsm->lane_allocator()->reset_metrics();

  /*
   * Run the FSM and apply steering forces if normal operation, otherwise handle
   * abnormal operation state.
   */
  supervisor()->run();
  ndc_uuid_pop();
} /* control_step() */

RCPPSW_WRAP_DEF_OVERRIDE(fcrw_bst_controller,
                         block_placement_intent,
                         *m_fsm,
                         const);

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
RCPPSW_WRAP_DEF_OVERRIDE(fcrw_bst_controller, goal_acquired, *m_fsm, const);
RCPPSW_WRAP_DEF_OVERRIDE(fcrw_bst_controller, entity_acquired_id, *m_fsm, const);
RCPPSW_WRAP_DEF_OVERRIDE(fcrw_bst_controller,
                         is_exploring_for_goal,
                         *m_fsm,
                         const);
RCPPSW_WRAP_DEF_OVERRIDE(fcrw_bst_controller, acquisition_goal, *m_fsm, const);
RCPPSW_WRAP_DEF_OVERRIDE(fcrw_bst_controller, block_transport_goal, *m_fsm, const);
RCPPSW_WRAP_DEF_OVERRIDE(fcrw_bst_controller, acquisition_loc3D, *m_fsm, const);
RCPPSW_WRAP_DEF_OVERRIDE(fcrw_bst_controller, vector_loc3D, *m_fsm, const);
RCPPSW_WRAP_DEF_OVERRIDE(fcrw_bst_controller, explore_loc3D, *m_fsm, const);

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

using namespace argos; // NOLINT

REGISTER_CONTROLLER(fcrw_bst_controller, "fcrw_bst_controller");

RCPPSW_WARNING_DISABLE_POP()

NS_END(controller, prism);
