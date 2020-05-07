/**
 * \file base_ct_block_place_interactor.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#ifndef INCLUDE_SILICON_SUPPORT_BASE_CT_BLOCK_PLACE_INTERACTOR_HPP_
#define INCLUDE_SILICON_SUPPORT_BASE_CT_BLOCK_PLACE_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/arena/operations/free_block_drop.hpp"
#include "cosm/tv/temporal_penalty.hpp"

#include "silicon/fsm/construction_acq_goal.hpp"
#include "silicon/structure/ct_manager.hpp"
#include "silicon/support/mpl/ct_block_place_spec.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class base_ct_block_place_interactor
 * \ingroup support depth2
 *
 * \brief Handles a robot's (possible) placement of a block onto a structure, if
 * the conditions necessary for the event are met, updating the robot and the
 * target structure as needed.
 */
template <typename TController, typename TControllerSpecMap>
class base_ct_block_place_interactor : public rer::client<base_ct_block_place_interactor<TController, TControllerSpecMap>> {
 public:
  using controller_spec = typename boost::mpl::at<TControllerSpecMap,
                                                  TController>::type;
  using interactor_status_type = typename controller_spec::interactor_status_type;
  using robot_block_place_visitor_type = typename controller_spec::robot_block_place_visitor_type;
  base_ct_block_place_interactor(sstructure::ct_manager* const manager)
      : ER_CLIENT_INIT("silicon.support.base_ct_block_place_interactor"),
        m_ct_manager(manager) {}

  base_ct_block_place_interactor(
      base_ct_block_place_interactor&&) = default;

  /* Not copy-constructible/assignable by default. */
  base_ct_block_place_interactor(
      const base_ct_block_place_interactor&) = delete;
  base_ct_block_place_interactor& operator=(
      const base_ct_block_place_interactor&) = delete;

  /**
   * \brief Determine if the robot has acquired its goal; this cannot be done in
   * a project-agnostic way, because it may require comparison of a controller's
   * acquisition_goal() function with a specific integer (for example).
   */
  virtual bool robot_goal_acquired(const TController& controller) const = 0;

  /**
   * \brief Callback to update controller bookkeeping (if needed) just prior to
   * it being visited by the structure block placement event.
   */
  virtual void robot_previsit_hook(TController&,
                                   const ctv::temporal_penalty&) const {}

  /**
   * \brief The actual handling function for interactions.
   *
   * \param controller The controller to handle interactions for.
   * \param t The current timestep.
   */
  interactor_status_type operator()(TController& controller,
                                    const rtypes::timestep& t) {
    auto* handler = m_ct_manager->placement_handler(controller.perception()->target());
    /* robot not currently on a structure */
    if (nullptr == handler) {
      return;
    }

    if (handler->is_serving_penalty(controller)) {
      if (handler->is_penalty_satisfied(controller, t)) {
        process_block_place(controller);
        return interactor_status::ekCT_BLOCK_PLACE;
      }
      return interactor_status::ekNO_EVENT;
    }
  }

 private:
  /**
   * \brief Once the robot has served its placement penalty, handle the actual
   * block placement interaction between the robot and the target structure.
   */
  void process_block_place(TController& controller) {
    auto* handler = m_ct_manager->placement_handler(controller.perception()->target());
    const ctv::temporal_penalty& p = handler->penalty_next();
    ER_ASSERT(p.controller() == &controller,
              "Out of order block placement handling");
    ER_ASSERT(robot_goal_acquire(controller),
              "Controller not waiting for block placement");
    auto site = controller.block_placement_site();
    ER_ASSERT(site, "Controller not intending to place block");

    auto builder = m_ct_manager->builder(controller.perception()->target());
    auto target = m_ct_manager->target(controller.perception()->target());

    auto coord = site - target->origind();
    if (builder->block_placement_valid(controller.block_release(),
                                       coord,
                                       target->cell_spec_retrieve(coord))) {
      execute_block_place(controller, p);
    } else {
      ER_WARN("Controller block placement on target%s invalid",
              rcppsw::to_string(target->id().c_str()));
    }

    handler->penalty_remove(p);
    ER_ASSERT(!handler->is_serving_penalty(controller),
                "Multiple instances of same controller serving cache penalty");
  }

  /**
   * \brief Perform the actual block placement.
   */
  void execute_block_place(TController& controller,
                           const ctv::temporal_penalty& penalty) {
    robot_block_place_visitor_type rplace_op(controller.id());
    auto builder = m_ct_manager->builder(controller.perception()->target());
    auto structure = m_ct_manager->target(controller.perception()->target());
    auto site = controller.intended_placement();
    auto coord = site - structure->origind();

    /* update bookkeeping */
    robot_previsit_hook(controller, penalty);

    /* place the block! */
    ER_ASSERT(builder->place_block(controller.block_release(),
                                   coord,
                                   structure->cell_spec_retrieve(coord)),
              "Failed to place block on structure%d@%s",
              structure->id().v(),
              coord.to_str().c_str());

    rplace_op.visit(controller);
  }

  /* clang-format off */
  sstructure::ct_manager*const m_ct_manager;
  /* clang-format on */
};

NS_END(support, silicon);

#endif /* INCLUDE_SILICON_SUPPORT_BASE_CT_BLOCK_PLACE_INTERACTOR_HPP_ */
