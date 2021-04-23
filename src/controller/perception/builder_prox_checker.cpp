/**
 * \file builder_prox_checker.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include "silicon/controller/perception/builder_prox_checker.hpp"

#include "rcppsw/utils/maskable_enum.hpp"

#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

#include "silicon/controller/perception/builder_perception_subsystem.hpp"
#include "silicon/fsm/calculators/lane_alignment.hpp"
#include "silicon/repr/colors.hpp"
#include "silicon/repr/construction_lane.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, controller, perception);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
builder_prox_type
builder_prox_checker::operator()(const srepr::construction_lane* lane,
                                 const srepr::fs_configuration& fs) const {
  builder_prox_type ret = builder_prox_type::ekNONE;
  ret |= trajectory_proximity(lane);
  ret |= frontier_set_proximity(fs, lane);
  return ret;
} /* robot_proximity() */

builder_prox_type builder_prox_checker::trajectory_proximity(
    const srepr::construction_lane* lane) const {
  auto readings = mc_sensing->blobs()->readings();
  auto rorientation = self_orientation();
  auto rpos = mc_sensing->rpos2D();
  auto prox = builder_prox_type::ekNONE;

  ER_TRACE("Robot orientation: pos_x=%d neg_x=%d pos_y=%d neg_y=%d",
           rorientation.pos_x,
           rorientation.neg_x,
           rorientation.pos_y,
           rorientation.neg_x);

  for (auto& r : readings) {
    /* If the robot is not a builder robot we ignore it */
    if (!is_builder_robot(r.color)) {
      continue;
    }

    /*
     * If the blob is more than 'tolerance' off from our current heading, then
     * it is not a robot we need to worry about.
     */
    if (r.vec.angle().unsigned_normalize() >
        sfsm::calculators::lane_alignment::kAZIMUTH_TOL) {
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
     * the relative offset of the other robot to get its absolute(ish)
     * position in the global reference frame.
     */
    rmath::vector2d other_rpos;
    lane_share_state sharing;
    if (rorientation.neg_x || rorientation.neg_y) {
      other_rpos = rpos - r.vec;
      sharing = share_state_calc(other_rpos, lane);
    } else {
      other_rpos = rpos + r.vec;
      sharing = share_state_calc(other_rpos, lane);
    }

    /* If the robot is not in our lane, ignore it */
    if (!sharing.is_shared) {
      continue;
    }

    /* If the robot does not share the ingress/egress lane with us, ignore it */
    if (!((sharing.self_in_ingress && sharing.other_in_ingress) ||
          (sharing.self_in_egress && sharing.other_in_egress))) {
      continue;
    }

    bool in_ct_zone = mc_sensing->ground()->detect("nest");
    if (in_ct_zone) {
      /*
       * Robot is in the middle of a hard turn in place and not align with any
       * of the 4 directions, so ignore any actual proximities until it
       * finishes.
       */
      if (!(rorientation.neg_x || rorientation.neg_y || rorientation.pos_x ||
            rorientation.pos_y)) {
        continue;
      }
    }
    if (r.vec.length() <= trajectory_prox_dist()) {
      prox |= builder_prox_type::ekTRAJECTORY;
    }
  } /* for(&r..) */
  return prox;
} /* trajectory_proximity() */

builder_prox_type builder_prox_checker::frontier_set_proximity(
    const srepr::fs_configuration& fs,
    const srepr::construction_lane* lane) const {
  auto readings = mc_sensing->blobs()->readings();
  auto rorientation = self_orientation();
  auto prox = builder_prox_type::ekNONE;

  ER_TRACE("Robot orientation: pos_x=%d neg_x=%d pos_y=%d neg_y=%d",
           rorientation.pos_x,
           rorientation.neg_x,
           rorientation.pos_y,
           rorientation.neg_x);

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

    auto rpos = mc_sensing->rpos2D();
    lane_share_state sharing;

    if (rorientation.neg_x || rorientation.neg_y) {
      sharing = share_state_calc(rpos - r.vec, lane);
    } else {
      sharing = share_state_calc(rpos + r.vec, lane);
    }

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
    if (sharing.self_in_egress) {
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
    if (srepr::fs_configuration::ekNONE == fs) {
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
  auto cellsize = mc_perception->nearest_ct()->block_unit_dim();
  return std::sqrt(cellsize * kTRAJECTORY_PROX_CELLS);
} /* trajectory_prox_dist() */

double builder_prox_checker::frontier_set_prox_dist(void) const {
  auto cellsize = mc_perception->nearest_ct()->block_unit_dim();
  return std::sqrt(cellsize * kFRONTIER_SET_PROX_CELLS);
} /* frontier_set_prox_dist() */

builder_prox_checker::lane_share_state builder_prox_checker::share_state_calc(
    const rmath::vector2d& other_rpos,
    const srepr::construction_lane* lane) const {
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
  auto self_dpos = mc_sensing->dpos2D();

  auto ct = mc_perception->nearest_ct();
  auto self_vcoord2D = ct->to_vcoord2D(self_dpos);
  auto cellsize = ct->block_unit_dim();

  auto ingress = lane->geometry().ingress_virt();
  auto egress = lane->geometry().egress_virt();

  double diff = 0;
  if (rmath::radians::kZERO == lane->orientation()) {
    diff = other_rpos.y() - self_rpos.y();
    ret.self_in_ingress = ingress.offset().y() == self_vcoord2D.offset().y();
    ret.self_in_egress = egress.offset().y() == self_vcoord2D.offset().y();
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation()) {
    diff = self_rpos.x() - other_rpos.x();
    ret.self_in_ingress = ingress.offset().x() == self_vcoord2D.offset().x();
    ret.self_in_egress = egress.offset().x() == self_vcoord2D.offset().x();
  } else if (rmath::radians::kPI == lane->orientation()) {
    diff = self_rpos.y() - other_rpos.y();
    ret.self_in_ingress = ingress.offset().y() == self_vcoord2D.offset().y();
    ret.self_in_egress = egress.offset().y() == self_vcoord2D.offset().y();

  } else if (rmath::radians::kTHREE_PI_OVER_TWO == lane->orientation()) {
    diff = other_rpos.x() - self_rpos.x();
    ret.self_in_ingress = ingress.offset().x() == self_vcoord2D.offset().x();
    ret.self_in_egress = egress.offset().x() == self_vcoord2D.offset().x();
  }

  if (ret.self_in_ingress) {
    ret.other_in_ingress = RCPPSW_IS_BETWEEN(std::fabs(diff), 0, cellsize * 0.5);
    ret.other_in_egress = RCPPSW_IS_BETWEEN(diff, cellsize * 0.5, cellsize * 1.5);
  } else {
    ret.other_in_ingress =
        RCPPSW_IS_BETWEEN(diff, -cellsize * 0.5, -cellsize * 1.5);
    ret.other_in_egress = RCPPSW_IS_BETWEEN(std::fabs(diff), 0, cellsize * 0.5);
  }
  ret.is_shared =
      (ret.self_in_ingress && (ret.other_in_ingress || ret.other_in_egress)) ||
      (ret.self_in_egress && (ret.other_in_egress || ret.other_in_ingress));

  ER_TRACE("Lane sharing: rpos=%s other=%s",
           rcppsw::to_string(self_rpos).c_str(),
           rcppsw::to_string(other_rpos).c_str());

  ER_TRACE("Lane sharing: self_ingress=%d self_egress=%d other_ingress=%d "
           "other_egress=%d",
           ret.self_in_ingress,
           ret.self_in_egress,
           ret.other_in_ingress,
           ret.other_in_egress);
  return ret;
} /* share_state_calc() */

bool builder_prox_checker::is_builder_robot(const rutils::color& color) const {
  auto colors = srepr::colors::ct();
  return colors.end() != std::find(colors.begin(), colors.end(), color);
} /* is_builder_robot() */

builder_prox_checker::robot_compass_orientation
builder_prox_checker::self_orientation(void) const {
  auto robot_azimuth = mc_sensing->azimuth();

  rmath::range<rmath::radians> kRobotPosXHeading{
    rmath::radians::kZERO - sfsm::calculators::lane_alignment::kAZIMUTH_TOL,
    rmath::radians::kZERO + sfsm::calculators::lane_alignment::kAZIMUTH_TOL
  };

  rmath::range<rmath::radians> kRobotNegXHeading{
    rmath::radians::kPI - sfsm::calculators::lane_alignment::kAZIMUTH_TOL,
    rmath::radians::kPI + sfsm::calculators::lane_alignment::kAZIMUTH_TOL
  };

  rmath::range<rmath::radians> kRobotPosYHeading{
    rmath::radians::kPI_OVER_TWO -
        sfsm::calculators::lane_alignment::kAZIMUTH_TOL,
    rmath::radians::kPI_OVER_TWO + sfsm::calculators::lane_alignment::kAZIMUTH_TOL
  };

  rmath::range<rmath::radians> kRobotNegYHeading{
    rmath::radians::kTHREE_PI_OVER_TWO -
        sfsm::calculators::lane_alignment::kAZIMUTH_TOL,
    rmath::radians::kTHREE_PI_OVER_TWO +
        sfsm::calculators::lane_alignment::kAZIMUTH_TOL
  };
  return { kRobotPosXHeading.contains(robot_azimuth.unsigned_normalize()),
           kRobotNegXHeading.contains(robot_azimuth.unsigned_normalize()),
           kRobotPosYHeading.contains(robot_azimuth.unsigned_normalize()),
           kRobotNegYHeading.contains(robot_azimuth.unsigned_normalize()) };
} /* self_orientation() */

NS_END(perception, controller, silicon);
