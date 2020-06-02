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
#include <utility>

#include "cosm/arena/operations/free_block_drop.hpp"
#include "cosm/repr/cube_block3D.hpp"
#include "cosm/repr/ramp_block3D.hpp"
#include "cosm/tv/temporal_penalty.hpp"
#include "cosm/arena/operations/nest_block_process.hpp"

#include "silicon/fsm/construction_acq_goal.hpp"
#include "silicon/structure/ct_manager.hpp"
#include "silicon/structure/structure3D_builder.hpp"
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
class base_ct_block_place_interactor
    : public rer::client<
          base_ct_block_place_interactor<TController, TControllerSpecMap>> {
 public:
  using controller_spec =
      typename boost::mpl::at<TControllerSpecMap, TController>::type;
  using interactor_status_type =
      typename controller_spec::interactor_status_type;
  using robot_block_place_visitor_type =
      typename controller_spec::robot_block_place_visitor_type;
  using penalty_handler_type = typename controller_spec::penalty_handler_type;
  using metrics_agg_type = typename controller_spec::metrics_agg_type;
  using arena_map_type = typename controller_spec::arena_map_type;

  base_ct_block_place_interactor(sstructure::ct_manager* const manager,
                                 arena_map_type* const arena_map,
                                 metrics_agg_type* const metrics_agg)
      : ER_CLIENT_INIT("silicon.support.base_ct_block_place_interactor"),
        m_ct_manager(manager),
        m_arena_map(arena_map),
        m_metrics_agg(metrics_agg) {}

  base_ct_block_place_interactor(base_ct_block_place_interactor&&) = default;

  /* Not copy-constructible/assignable by default. */
  base_ct_block_place_interactor(const base_ct_block_place_interactor&) = delete;
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
   * \brief If the robot is not currently serving a penalty, then this callback
   * is used to attempt to initialized one. It is needed because the penalty
   * handler type is unknown, and therefore so are the arguments needed for
   * initializing the penalty, beyond the controller and the timestep.
   */
  virtual void robot_penalty_init(const TController& controller,
                                  const rtypes::timestep& t,
                                  penalty_handler_type* handler) = 0;

  /**
   * \brief The actual handling function for interactions.
   *
   * \param controller The controller to handle interactions for.
   * \param t The current timestep.
   */
  interactor_status_type operator()(TController& controller,
                                    const rtypes::timestep& t) {
    /* Robot does not currently know of any construction targets */
    auto* ct = controller.perception()->nearest_ct();
    if (nullptr == ct) {
      return interactor_status::ekNO_EVENT;
    }

    auto* handler = m_ct_manager->placement_handler(ct->id());
    if (handler->is_serving_penalty(controller)) {
      if (handler->is_penalty_satisfied(controller, t)) {
        process_block_place(controller, handler, t);
        return interactor_status::ekCT_BLOCK_PLACE;
      }
    } else {
      robot_penalty_init(controller, t, handler);
    }
    return interactor_status::ekNO_EVENT;
  }

 private:
  /**
   * \brief Once the robot has served its placement penalty, handle the actual
   * block placement interaction between the robot and the target structure.
   */
  void process_block_place(TController& controller,
                           penalty_handler_type* handler,
                           const rtypes::timestep& t) {
    const ctv::temporal_penalty& p = handler->penalty_next();
    ER_ASSERT(p.controller() == &controller,
              "Out of order block placement handling");
    ER_ASSERT(robot_goal_acquired(controller),
              "Controller not waiting for block placement");
    auto intent = controller.block_placement_intent();
    ER_ASSERT(intent, "Controller not intending to place block");
    auto* ct = controller.perception()->nearest_ct();
    auto builder = m_ct_manager->builder(ct->id());
    auto target = m_ct_manager->target(ct->id());

    if (builder->block_placement_valid(to_variant(controller.block()),
                                       intent->site,
                                       intent->orientation)) {
      execute_block_place(controller, p, *intent, t);
    } else {
      ER_WARN("Block placement on target%s@%s, orientation=%s invalid",
              rcppsw::to_string(target->id()).c_str(),
              rcppsw::to_string(intent->site).c_str(),
              rcppsw::to_string(intent->orientation).c_str());
    }

    handler->penalty_remove(p);
    ER_ASSERT(!handler->is_serving_penalty(controller),
              "Multiple instances of same controller serving cache penalty");
  }

  /**
   * \brief Perform the actual block placement.
   */
  void execute_block_place(TController& controller,
                           const ctv::temporal_penalty& penalty,
                           const fsm::block_placer::placement_intent& intent,
                           const rtypes::timestep& t) {
    robot_block_place_visitor_type rplace_op(controller.entity_id(),
                                             controller.block()->dpos3D());
    caops::nest_block_process_visitor aproc_op(controller.block()->id(),
                                               t);

    auto* ct = controller.perception()->nearest_ct();
    auto builder = m_ct_manager->builder(ct->id());
    auto structure = m_ct_manager->target(ct->id());

    /*
     * We have to do this asynchronous to the rest of metric collection, because
     * the nest block drop event resets block metrics.
     */
    controller.block()->md()->dest_drop_time(t);
    m_metrics_agg->collect_from_block(controller.block());

    /* update controller bookkeeping */
    robot_previsit_hook(controller, penalty);

    /* place the block! */
    auto block = controller.block_release();
    ER_ASSERT(builder->place_block(std::move(block),
                                   intent.site,
                                   intent.orientation),
              "Failed to place block on target%s@%s, orientation=%s",
              rcppsw::to_string(structure->id()).c_str(),
              rcppsw::to_string(intent.site).c_str(),
              rcppsw::to_string(intent.orientation).c_str());
    /*
     * Distribute the block so that the # blocks available to robots in the
     * arena stays the same after the build process "consumes" one.
     */
    aproc_op.visit(*m_arena_map);

    /* update controller with block placement */
    rplace_op.visit(controller);
  }

  crepr::block3D_variant to_variant(crepr::base_block3D* block) const {
    if (crepr::block_type::ekRAMP == block->md()->type()) {
      return crepr::block3D_variant(static_cast<crepr::ramp_block3D*>(block));
    } else if (crepr::block_type::ekCUBE == block->md()->type()) {
      return crepr::block3D_variant(static_cast<crepr::cube_block3D*>(block));
    } else {
      ER_FATAL_SENTINEL("Bad block type %d",
                        rcppsw::as_underlying(block->md()->type()));
    }
  }
  /* clang-format off */
  sstructure::ct_manager*const m_ct_manager;
  arena_map_type* const        m_arena_map;
  metrics_agg_type * const     m_metrics_agg;
  /* clang-format on */
};

NS_END(support, silicon);

#endif /* INCLUDE_SILICON_SUPPORT_BASE_CT_BLOCK_PLACE_INTERACTOR_HPP_ */
