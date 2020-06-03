/**
 * \file fs_acq_checker.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/fsm/fs_acq_checker.hpp"

#include "rcppsw/math/radians.hpp"

#include "cosm/ds/cell3D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

#include "silicon/controller/perception/builder_perception_subsystem.hpp"
#include "silicon/repr/construction_lane.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
fs_acq_checker::fs_acq_checker(
    const csubsystem::sensing_subsystemQ3D* sensing,
    const scperception::builder_perception_subsystem* perception)
    : ER_CLIENT_INIT("silicon.fsm.fs_acq_checker"),
      mc_sensing(sensing),
      mc_perception(perception) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
stygmergic_configuration fs_acq_checker::operator()(
    const srepr::construction_lane* lane) const {

  auto* los = mc_perception->los();
  ER_ASSERT(nullptr != los,
            "Frontier set acquisition checker called without LOS");

  ER_TRACE("robot position: %s/%s LOS LL origin: %s",
           rcppsw::to_string(mc_sensing->rpos3D()).c_str(),
           rcppsw::to_string(mc_sensing->dpos3D()).c_str(),
           rcppsw::to_string(los->abs_ll()).c_str());

  /*
   * \todo Right now this assumes cube blocks, and is only correct for
   * such. This will be fixed once the most basic construction has been
   * validated.
   */
  auto los_cells = los_cells_calc(lane);
  return configuration_calc(los_cells, lane);
} /* operator()() */

fs_acq_checker::los_lane_cells fs_acq_checker::los_cells_calc(
    const srepr::construction_lane* lane) const {
  rmath::vector2z ingress_fs_cell;
  rmath::vector2z egress_fs_cell;
  los_lane_cells ret;
  auto* los = mc_perception->los();

  /*
   * Calculate which cell in the target the robot is currently in, then use
   * that to figure out the robot's relative location within the LOS.
   */
  auto* ct = mc_perception->nearest_ct();
  auto robot_ct_cell = (mc_sensing->dpos3D() -
                        ct->vorigind()) / ct->unit_dim_factor();
  ER_ASSERT(robot_ct_cell >= los->abs_ll(), "Robot CT cell not in LOS");
  auto robot_los_rel = (robot_ct_cell - los->abs_ll()).to_2D();

  /*
   * For all targets, the origin of the structure is ALWAYS in the lower left
   * corner, regardless of orientation. This is to make it line up nicely with
   * the LOS, which also always has its origin in the LL corner.
   */
  if (rmath::radians::kZERO == lane->orientation()) {
    /* We look in the NEGATIVE X direction for stygmergic configurations */
    ingress_fs_cell = robot_los_rel + rmath::vector2z(-kDETECT_CELL_DIST,
                                                      0);
    egress_fs_cell = robot_los_rel + rmath::vector2z(-kDETECT_CELL_DIST,
                                                     1);
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation()) {
    /* We look in the NEGATIVE Y direction for stygmergic configurations */
    ingress_fs_cell = robot_los_rel + rmath::vector2z(0,
                                                      -kDETECT_CELL_DIST);
    egress_fs_cell = robot_los_rel +
                     rmath::vector2z(0, -kDETECT_CELL_DIST) -
                     rmath::vector2z(1, 0);
  } else if (rmath::radians::kPI == lane->orientation()) {
    /* We look in the POSITIVE X direction for stygmergic configurations */
    ingress_fs_cell = robot_los_rel + rmath::vector2z(kDETECT_CELL_DIST,
                                                      0);
    egress_fs_cell = robot_los_rel +
                     rmath::vector2z(kDETECT_CELL_DIST, 0) -
                     rmath::vector2z(0, 1);
  } else if (rmath::radians::kTHREE_PI_OVER_TWO == lane->orientation()) {
    /* We look in the POSITIVE Y direction for stygmergic configurations  */
    ingress_fs_cell = robot_los_rel + rmath::vector2z(0,
                                                      kDETECT_CELL_DIST);
    egress_fs_cell = robot_los_rel + rmath::vector2z(1,
                                                     kDETECT_CELL_DIST);
  } else {
    ER_FATAL_SENTINEL("Bad lane orientation '%s'",
                      rcppsw::to_string(lane->orientation()).c_str());
  }
  ER_TRACE("ingress_fs_cell: %s egress_fs_cell: %s",
           rcppsw::to_string(ingress_fs_cell).c_str(),
           rcppsw::to_string(egress_fs_cell).c_str());

  ER_ASSERT(los->contains_rel(ingress_fs_cell),
            "LOS does not contain ingress lane frontier cell candidate %s",
            rcppsw::to_string(ingress_fs_cell).c_str());
  ER_ASSERT(los->contains_rel(egress_fs_cell),
            "LOS does not contain egress lane frontier cell candidate %s",
            rcppsw::to_string(egress_fs_cell).c_str());

  ret.ingress = &los->access(ingress_fs_cell);
  ret.egress = &los->access(egress_fs_cell);
  return ret;
} /* los_cells_calc() */

stygmergic_configuration fs_acq_checker::configuration_calc(
    const los_lane_cells& los_cells,
    const srepr::construction_lane* lane) const {
  bool ingress_has_block = los_cells.ingress->state_has_block();
  bool egress_has_block = los_cells.egress->state_has_block();

  if (ingress_has_block && egress_has_block) {
    ER_INFO("LANE_FILLED configuration encountered: ingress=%s,egress=%s",
            rcppsw::to_string(los_cells.ingress->loc()).c_str(),
            rcppsw::to_string(los_cells.egress->loc()).c_str());
    return stygmergic_configuration::ekLANE_FILLED;
  } else if (ingress_has_block && !egress_has_block) {
    ER_INFO("LANE_GAP_EGRESS configuration encountered: ingress=%s,egress=%s",
            rcppsw::to_string(los_cells.ingress->loc()).c_str(),
            rcppsw::to_string(los_cells.egress->loc()).c_str());
    return stygmergic_configuration::ekLANE_GAP_EGRESS;
  } else if (!ingress_has_block && egress_has_block) {
    ER_INFO("LANE_GAP_INGRESS configuration encountered: ingress=%s,egress=%s",
            rcppsw::to_string(los_cells.ingress->loc()).c_str(),
            rcppsw::to_string(los_cells.egress->loc()).c_str());
    return stygmergic_configuration::ekLANE_GAP_INGRESS;
  }

  /* possibly empty lane encountered--need to check to see for sure */
  bool empty = false;
  auto* ct = mc_perception->nearest_ct();
  if (rmath::radians::kZERO == lane->orientation()) {
    empty = (ct->vshell_sized() == los_cells.ingress->loc().x() &&
             ct->vshell_sized() == los_cells.egress->loc().x());
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation()) {
    empty = (ct->vshell_sized() == los_cells.ingress->loc().y() &&
             ct->vshell_sized() == los_cells.egress->loc().y());
  } else if (rmath::radians::kPI == lane->orientation()) {
    empty = (ct->vshell_sized() + ct->bbd().x() - 1 == los_cells.ingress->loc().x() &&
             ct->vshell_sized() + ct->bbd().x() - 1 == los_cells.egress->loc().x());
  } else if (rmath::radians::kTHREE_PI_OVER_TWO == lane->orientation()) {
    empty = (ct->vshell_sized() + ct->bbd().y() - 1 == los_cells.ingress->loc().y() &&
             ct->vshell_sized() + ct->bbd().y() - 1 == los_cells.egress->loc().y());
  } else {
    ER_FATAL_SENTINEL("Bad lane orientation '%s'",
                      rcppsw::to_string(lane->orientation()).c_str());
  }
  if (empty) {
    ER_INFO("LANE_EMPTY configuration encountered: ingress=%s,egress=%s",
            rcppsw::to_string(los_cells.ingress->loc()).c_str(),
            rcppsw::to_string(los_cells.egress->loc()).c_str());
    return stygmergic_configuration::ekLANE_EMPTY;
  }
  return stygmergic_configuration::ekNONE;
} /* configuration_calc() */

NS_END(fsm, silicon);
