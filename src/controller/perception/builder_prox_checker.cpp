/**
 * \file builder_prox_checker.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include "prism/controller/perception/builder_prox_checker.hpp"

#include "rcppsw/utils/maskable_enum.hpp"

#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

#include "prism/controller/perception/builder_perception_subsystem.hpp"
#include "prism/fsm/calculators/lane_alignment.hpp"
#include "prism/repr/colors.hpp"
#include "prism/repr/construction_lane.hpp"
#include "prism/algorithm/constants.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, controller, perception);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
builder_prox_type
builder_prox_checker::operator()(const prepr::construction_lane* lane,
                                 const prepr::fs_configuration& fs) const {
  builder_prox_type ret = builder_prox_type::ekNONE;
  ret |= trajectory_proximity(lane);
  ret |= frontier_set_proximity(fs, lane);
  return ret;
} /* robot_proximity() */

builder_prox_type builder_prox_checker::trajectory_proximity(
    const prepr::construction_lane* lane) const {
  /*
   * Get readings with angles relative to the robots current heading, rather
   * than PI=0 (default).
   */
  auto readings = mc_sensing->blobs()->readings(mc_sensing->azimuth());
  auto self_orientation = mc_perception->self_compass_orientation(mc_sensing->azimuth());
  auto self_rpos = mc_sensing->rpos2D();
  auto prox = builder_prox_type::ekNONE;

  auto self_traversal = mc_perception->lane_traversal_state(lane, self_rpos);

  ER_DEBUG("Self lane traversal: pos=%s in_ingress=%d in_egress=%d",
           rcppsw::to_string(self_rpos).c_str(),
           self_traversal.in_ingress,
           self_traversal.in_egress);

  ER_DEBUG("Self orientation: pos_x=%d neg_x=%d pos_y=%d neg_y=%d",
           self_orientation.pos_x,
           self_orientation.neg_x,
           self_orientation.pos_y,
           self_orientation.neg_y);

  /*
   * Robot is in the middle of a hard turn in place and not align with any
   * of the 4 directions, so ignore any actual proximities until it
   * finishes.
   */
  if (!(self_orientation.neg_x || self_orientation.neg_y ||
        self_orientation.pos_x || self_orientation.pos_y)) {
    return prox;
  }

  for (auto& r : readings) {
    /* If the robot is not a builder robot we ignore it */
    if (!is_builder_robot(r.color)) {
      continue;
    }

    /*
     * If the blob is more than 'tolerance' off from our current heading, then
     * it is not a robot we need to worry about.
     */
    if (!mc_perception->is_aligned_with(r.vec.angle(), mc_sensing->azimuth())) {
      continue;
    }

    /*
     * If the robot is behind us (relative to what direction we are facing) we
     * ignore it, as that is a robot which will have to wait on US, because we
     * are in front.
     */
    if (mc_perception->is_behind_self(r.vec, lane)) {
      continue;
    }

    /*
     * Depending on which way we are facing we need to either add or subtract
     * the relative offset of the other robot to get its absolute position in
     * the global reference frame.
     */

    auto sharing = share_state_calc(self_rpos + r.vec, lane);

    /* If the robot is not in our lane, ignore it */
    if (!sharing.is_shared) {
      continue;
    }

    /* If the robot does not share the ingress/egress lane with us, ignore it */
    if (!((sharing.self.in_ingress && sharing.other.in_ingress) ||
          (sharing.self.in_egress && sharing.other.in_egress))) {
      continue;
    }

    if (r.vec.length() <= trajectory_prox_dist()) {
      prox |= builder_prox_type::ekTRAJECTORY;
    }
  } /* for(&r..) */
  return prox;
} /* trajectory_proximity() */

builder_prox_type builder_prox_checker::frontier_set_proximity(
    const prepr::fs_configuration& fs,
    const prepr::construction_lane* lane) const {
  /*
   * Get readings with angles relative to the robots current heading, rather
   * than PI=0 (default).
   */
  auto readings = mc_sensing->blobs()->readings(mc_sensing->azimuth());
  auto self_orientation = mc_perception->self_compass_orientation(mc_sensing->azimuth());
  auto prox = builder_prox_type::ekNONE;
  auto self_rpos = mc_sensing->rpos2D();

  auto self_traversal = mc_perception->lane_traversal_state(lane, self_rpos);

  ER_DEBUG("Self lane traversal: pos=%s in_ingress=%d in_egress=%d",
           rcppsw::to_string(self_rpos).c_str(),
           self_traversal.in_ingress,
           self_traversal.in_egress);

  ER_DEBUG("Robot orientation: pos_x=%d neg_x=%d pos_y=%d neg_y=%d",
           self_orientation.pos_x,
           self_orientation.neg_x,
           self_orientation.pos_y,
           self_orientation.neg_y);

  /*
   * Robot is in the middle of a hard turn in place and not aligned with any of
   * the 4 directions, so ignore any actual proximities until it finishes.
   */
  if (!(self_orientation.neg_x || self_orientation.neg_y
        || self_orientation.pos_x || self_orientation.pos_y)) {
    return prox;
  }

  /*
   * If we are not aligned with our lane we are executing a hard turn adjacent
   * to the frontier set as we acquire our placement site, so by definition
   * there are no robots ahead of us.
   */
  if (!mc_perception->self_lane_aligned(lane)) {
    return prox;
  }

  for (auto& r : readings) {
    /* If the robot is not a builder robot we ignore it */
    if (!is_builder_robot(r.color)) {
      continue;
    }

    /*
     * If the robot is behind us (relative to what direction we are facing) we
     * ignore it, as that is a robot which will have to wait on US, because we
     * are in front.
     */
    if (mc_perception->is_behind_self(r.vec, lane)) {
      continue;
    }


    auto sharing = share_state_calc(self_rpos + r.vec, lane);

    /* If the robot is not in our lane, ignore it */
    if (!sharing.is_shared) {
      continue;
    }

    /*
     * If we get here, then there is another robot sharing our lane which is in
     * front of us. If we are in the egress lane, then we have already placed
     * our block on the structure (or aborted placement and are leaving), and so
     * builder robot proximities cannot happen (we must be in the ingress lane
     * for them to occur).
     */
    if (sharing.self.in_egress) {
      continue;
    }

    /*
     * If we get here, then we are sharing the lane with another robot who is in
     * front of us, and we are in the ingress lane. Regardless of which lane the
     * other robot is in, we only consider it a builder proximity if we have
     * acquired the frontier set. Prior to that we are just in the middle of
     * traversing the ingress lane (which the robot in front of us is presumably
     * also doing), so no builder proximity.
     *
     */
    if (prepr::fs_configuration::ekNONE == fs) {
      continue;
    }
    /*
     * We are in the ingress lane, are sharing the lane with another robot ahead
     * of us. Don't follow too closely!
     */
    if (r.vec.length() <= frontier_set_prox_dist()) {
      prox |= builder_prox_type::ekFRONTIER_SET;
    }
  } /* for(&r..) */
  return prox;
} /* frontier_set_proximity() */

double builder_prox_checker::trajectory_prox_dist(void) const {
  auto cellsize = mc_perception->nearest_ct()->block_unit_dim().v();
  return std::sqrt(cellsize * paconstants::kCT_TRAJECTORY_PROX_CELLS);
} /* trajectory_prox_dist() */

double builder_prox_checker::frontier_set_prox_dist(void) const {
  auto cellsize = mc_perception->nearest_ct()->block_unit_dim().v();
  return std::sqrt(cellsize * paconstants::kCT_FRONTIER_SET_PROX_CELLS);
} /* frontier_set_prox_dist() */

builder_prox_checker::lane_share_state builder_prox_checker::share_state_calc(
    const rmath::vector2d& other_rpos,
    const prepr::construction_lane* lane) const {
  lane_share_state ret;

  /*
   * We are not on the structure/have allocated a lane yet, and so by definition
   * are not sharing it with another robot.
   */
  if (nullptr == lane) {
    return ret;
  }

  /*
   * We only consider the projection of the robot's 3D position on the 2D plane,
   * since that is how construction lanes are defined.
   */
  auto self_rpos = mc_sensing->rpos2D();

  ret.self = mc_perception->lane_traversal_state(lane, self_rpos);
  ret.other = mc_perception->lane_traversal_state(lane, other_rpos);

  ret.is_shared =
      (ret.self.in_ingress && (ret.other.in_ingress || ret.other.in_egress)) ||
      (ret.self.in_egress && (ret.other.in_egress || ret.other.in_ingress));

  ER_TRACE("Lane sharing: rpos=%s other=%s",
           rcppsw::to_string(self_rpos).c_str(),
           rcppsw::to_string(other_rpos).c_str());

  ER_TRACE("Lane sharing: self.ingress=%d self.egress=%d other.ingress=%d "
           "other_egress=%d",
           ret.self.in_ingress,
           ret.self.in_egress,
           ret.other.in_ingress,
           ret.other.in_egress);
  return ret;
} /* share_state_calc() */

bool builder_prox_checker::is_builder_robot(const rutils::color& color) const {
  auto colors = prepr::colors::ct();
  return colors.end() != std::find(colors.begin(), colors.end(), color);
} /* is_builder_robot() */


NS_END(perception, controller, prism);
