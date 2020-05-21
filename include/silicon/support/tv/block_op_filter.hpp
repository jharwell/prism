/**
 * \file block_op_filter.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * This file is part of FORDYCA.
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

#ifndef INCLUDE_SILICON_SUPPORT_TV_BLOCK_OP_FILTER_HPP_
#define INCLUDE_SILICON_SUPPORT_TV_BLOCK_OP_FILTER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <boost/optional.hpp>

#include "cosm/arena/base_arena_map.hpp"

#include "silicon/support/tv/block_op_src.hpp"
#include "silicon/support/tv/op_filter_status.hpp"
#include "silicon/fsm/construction_acq_goal.hpp"
#include "silicon/fsm/construction_transport_goal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, support, tv);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class block_op_filter
 * \ingroup support tv
 *
 * \brief The filter for block operation penalties for robots, determining if a
 * given robot has satisfied all the conditions necessary for triggering a given
 * type of operation.
 */
class block_op_filter : public rer::client<block_op_filter> {
 public:
  explicit block_op_filter(const carena::base_arena_map* const map)
      : ER_CLIENT_INIT("silicon.support.tv.block_op_filter"),
        mc_map(map) {}

  ~block_op_filter(void) override = default;
  block_op_filter& operator=(const block_op_filter&) = delete;
  block_op_filter(const block_op_filter&) = delete;

  /**
   * \brief Filters out controllers that actually are not eligible to start
   * serving penalties.
   *
   * \return (\c TRUE, penalty_status) iff the controller should be filtered out
   * and the reason why. (\c FALSE, -1) otherwise.
   */
  template <typename TControllerType>
  op_filter_status operator()(const TControllerType& controller,
                              block_op_src src) {
    /*
     * If the robot has not acquired a block, or thinks it has but actually has
     * not, nothing to do. If a robot is carrying a block but is still
     * transporting it (even if it IS currently in the nest), nothing to do.
     */
    switch (src) {
      case block_op_src::ekARENA_PICKUP:
        return free_pickup_filter(controller);
      case block_op_src::ekCT_BLOCK_MANIP:
        return structure_placement_filter(controller);
      default:
        ER_FATAL_SENTINEL("Unhandled penalty type %d", static_cast<int>(src));
    } /* switch() */
    return op_filter_status{};
  }

 private:
  /**
   * \brief Filter out spurious penalty initializations for free block pickup
   * (i.e. controller not ready/not intending to pickup a free block).
   *
   */
  template <typename TControllerType>
  op_filter_status free_pickup_filter(const TControllerType& controller) const {
    /*
     * @todo This should not need the arena map header file, but I don't have a
     * base controller for SILICON yet, so I can't create a
     * utils::robot_on_block like in FORDYCA that keeps that header out of this
     * file.
     */
    if (!(controller.goal_acquired() &&
          fsm::construction_acq_goal::ekFORAGING_BLOCK == controller.acquisition_goal())) {
      return op_filter_status::ekROBOT_INTERNAL_UNREADY;
    }

    auto id = mc_map->robot_on_block(controller.rpos2D(),
                                     controller.entity_acquired_id());
    if (rtypes::constants::kNoUUID == id) {
      return op_filter_status::ekROBOT_NOT_ON_BLOCK;
    }

    return op_filter_status::ekSATISFIED;
  }

  /**
   * \brief Filter out spurious penalty initializations for structure block
   * placement (i.e. controller not ready/not intending to place a block on the
   * structure).
   */
  template <typename TControllerType>
  op_filter_status structure_placement_filter(const TControllerType& controller) const {
    if (controller.in_nest() && controller.goal_acquired() &&
        fsm::construction_acq_goal::ekCT_BLOCK_PLACEMENT_SITE == controller.acquisition_goal()) {
      return op_filter_status::ekSATISFIED;
    }
    return op_filter_status::ekROBOT_INTERNAL_UNREADY;
  }

  /* clang-format off */
  const carena::base_arena_map* const mc_map;
  /* clang-format on */
};
NS_END(tv, support, silicon);

#endif /* INCLUDE_SILICON_SUPPORT_TV_BLOCK_OP_FILTER_HPP_ */
