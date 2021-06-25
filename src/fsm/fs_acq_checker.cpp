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
srepr::fs_configuration
fs_acq_checker::operator()(const srepr::construction_lane* lane) const {
  const auto* los = mc_perception->los();
  const auto* ct = mc_perception->nearest_ct();

  if (nullptr == los) {
    return srepr::fs_configuration::ekNONE;
  }

  ER_TRACE("Robot position: %s/%s LOS LL origin: %s/%s",
           rcppsw::to_string(mc_sensing->rpos3D()).c_str(),
           rcppsw::to_string(mc_sensing->dpos3D()).c_str(),
           rcppsw::to_string(los->abs_ll()).c_str(),
           rcppsw::to_string(rmath::zvec2dvec(los->abs_ll(),
                                              ct->grid_resolution().v())).c_str());

  /*
   * \todo Right now this assumes cube blocks, and is only correct for
   * such. This will be fixed once the most basic construction has been
   * validated.
   */
  auto ret = srepr::fs_configuration::ekNONE;
  for (size_t lookahead = 1; lookahead <= kDETECT_CELL_DIST_MAX; ++lookahead) {
    auto result = acq_result_calc(lane, lookahead);
    ret = configuration_calc(result, lane, los);

    /*
     * Give priority to configurations encountered closer to the robot: as soon
     * as we find one, don't look any further out.
     */
    if (srepr::fs_configuration::ekNONE != ret) {
      break;
    }
  } /* for(lookahead..) */
  return ret;
} /* operator()() */

fs_acq_checker::acq_result
fs_acq_checker::acq_result_calc(const srepr::construction_lane* lane,
                               size_t lookahead) const {
  acq_result ret;

  const auto* ct = mc_perception->nearest_ct();
  auto robot_ct_cell = ct->to_vcoord(mc_sensing->rpos3D());
  const auto* los = mc_perception->los();

  ER_ASSERT(robot_ct_cell.offset() >= los->abs_ll(),
            "Robot CT cell not in LOS");

  auto rpos = mc_sensing->rpos3D();
  auto dpos = mc_sensing->dpos3D();

  ret.positions = acq_positions_calc(rpos, lane, lookahead);

  auto ingress_arena_cell = rmath::dvec2zvec(ret.positions.ingress,
                                             mc_perception->arena_resolution().v());
  auto egress_arena_cell = rmath::dvec2zvec(ret.positions.egress,
                                            mc_perception->arena_resolution().v());
  auto ingress_ct_cell = ct->to_vcoord(ret.positions.ingress);
  auto egress_ct_cell = ct->to_vcoord(ret.positions.egress);

  auto ingress_los_cell = (ingress_ct_cell.offset() - los->abs_ll());
  auto egress_los_cell = (egress_ct_cell.offset() - los->abs_ll());

  auto robot_los_cell = (robot_ct_cell.offset() - los->abs_ll()).to_2D();

  ER_TRACE("Robot arena cell=%s,ingress arena cell=%s,egress arena cell=%s",
           rcppsw::to_string(dpos).c_str(),
           rcppsw::to_string(ingress_arena_cell).c_str(),
           rcppsw::to_string(egress_arena_cell).c_str());

  ER_TRACE("Robot CT cell=%s,ingress CT cell=%s,egress CT cell=%s",
           rcppsw::to_string(robot_ct_cell).c_str(),
           rcppsw::to_string(ingress_ct_cell).c_str(),
           rcppsw::to_string(egress_ct_cell).c_str());

  ER_TRACE("Robot LOS cell=%s,ingress LOS cell=%s,egress LOS cell=%s,lookahead=%zu",
           rcppsw::to_string(robot_los_cell).c_str(),
           rcppsw::to_string(ingress_los_cell).c_str(),
           rcppsw::to_string(egress_los_cell).c_str(),
           lookahead);

  /*
   * This is not an error because when we first enter the virtual shell of the
   * structure and get a LOS, our "lookahead" might exceed the size of the
   * LOS. In general once we are on the structure proper this should never
   * happen.
   */
  if (!los->contains_rel(ingress_los_cell) || !los->contains_rel(egress_los_cell)) {
    ER_CHECKW(los->contains_rel(ingress_los_cell),
              "LOS does not contain ingress lane frontier cell candidate %s",
              rcppsw::to_string(ingress_los_cell).c_str());
    ER_CHECKW(los->contains_rel(egress_los_cell),
              "LOS does not contain egress lane frontier cell candidate %s",
              rcppsw::to_string(egress_los_cell).c_str());
  } else {
    ret.cells.ingress = &los->access(ingress_los_cell);
    ret.cells.egress = &los->access(egress_los_cell);
  }

  ret.lookahead = lookahead;
  return ret;
} /* acq_result_calc() */

srepr::fs_configuration
fs_acq_checker::configuration_calc(const acq_result& result,
                                   const srepr::construction_lane* lane,
                                   const srepr::builder_los* los) const {
  if (nullptr != result.cells.ingress && nullptr != result.cells.egress) {
    bool ingress_has_block = result.cells.ingress->state_has_block();
    bool egress_has_block = result.cells.egress->state_has_block();

    if (ingress_has_block && egress_has_block) {
      ER_INFO("LANE_FILLED encountered: ingress=%s,egress=%s,lookahead=%zu",
              rcppsw::to_string(result.cells.ingress->loc()).c_str(),
              rcppsw::to_string(result.cells.egress->loc()).c_str(),
              result.lookahead);
      return srepr::fs_configuration::ekLANE_FILLED;
    } else if (ingress_has_block && !egress_has_block) {
      ER_INFO("LANE_GAP_EGRESS encountered: ingress=%s,egress=%s,lookahead=%zu",
              rcppsw::to_string(result.cells.ingress->loc()).c_str(),
              rcppsw::to_string(result.cells.egress->loc()).c_str(),
              result.lookahead);
      return srepr::fs_configuration::ekLANE_GAP_EGRESS;
    } else if (!ingress_has_block && egress_has_block) {
      ER_INFO("LANE_GAP_INGRESS encountered: ingress=%s,egress=%s,lookahead=%zu",
              rcppsw::to_string(result.cells.ingress->loc()).c_str(),
              rcppsw::to_string(result.cells.egress->loc()).c_str(),
              result.lookahead);
      return srepr::fs_configuration::ekLANE_GAP_INGRESS;
    }
  } else {
    /* possibly empty lane encountered--need to check to see for sure */
    if (acq_empty_lane(lane, los)) {
      ER_INFO("LANE_EMPTY encountered: ingress=%s,egress=%s,lookahead=%zu",
              rcppsw::to_string(result.positions.ingress).c_str(),
              rcppsw::to_string(result.positions.egress).c_str(),
              result.lookahead);
      return srepr::fs_configuration::ekLANE_EMPTY;
    }
  }

  return srepr::fs_configuration::ekNONE;
} /* configuration_calc() */

bool fs_acq_checker::acq_empty_lane(const srepr::construction_lane* lane,
                                    const srepr::builder_los* los) const {
  const auto* ct = mc_perception->nearest_ct();
  /*
   * The origin of the LOS is always in the LL corner, regardless of lane
   * orientation, which simplifies calculations of empty lane acquisition a
   * lot. The returned spans of the LOS are specified relative to the grid
   * origin (structure virtual origin), so we account for that here to get spans
   * relative to the arena origin.
   */
  auto xspan = los->xrspan().recenter(mc_sensing->rpos3D().x());
  auto yspan = los->yrspan().recenter(mc_sensing->rpos3D().y());

  if (rmath::radians::kZERO == lane->orientation()) {
    return xspan.ub() >= ct->voriginr().x() + ct->bbr().x() + ct->vshell_sizer().v();
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation()) {
    return yspan.ub() >= ct->voriginr().y() + ct->bbr().y() + ct->vshell_sizer().v();
  } else if (rmath::radians::kPI == lane->orientation()) {
    return xspan.lb() <= ct->voriginr().x() + ct->vshell_sizer().v();
  } else if (rmath::radians::kTHREE_PI_OVER_TWO == lane->orientation()) {
    return yspan.lb() <= ct->voriginr().y() + ct->vshell_sizer().v();
  } else {
    ER_FATAL_SENTINEL("Bad lane orientation '%s'",
                      rcppsw::to_string(lane->orientation()).c_str());
  }
  return false;
} /* acq_empty_lane() */

fs_acq_checker::acq_positions fs_acq_checker::acq_positions_calc(
    const rmath::vector3d& rpos,
    const srepr::construction_lane* lane,
    size_t lookahead) const {
  acq_positions ret;

  const auto* ct = mc_perception->nearest_ct();
  auto cell_size = ct->block_unit_dim();

  /*
   * For all targets, the origin of the structure is ALWAYS in the lower left
   * corner, regardless of orientation. This is to make it line up nicely with
   * the LOS, which also always has its origin in the LL corner.
   *
   * For all targets, we have to compute ingress/ingress by doing all our
   * calculations in real coordinates, and then converting to discrete ONLY
   * after the final value has been achieved. Otherwise, you run the risk of
   * off-by-one errors if the arena and/or ct resolution is != 1.0.
   */
  if (rmath::radians::kZERO == lane->orientation()) {
    /* We look in the POSITIVE X direction for stygmergic configurations */
    ret.ingress = rpos + rmath::vector3d((lookahead * cell_size), 0, 0);
    ret.egress = rpos + rmath::vector3d((lookahead * cell_size),
                                        -cell_size,
                                        0);
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation()) {
    /* We look in the POSITIVE Y direction for stygmergic configurations  */
    ret.ingress = rpos + rmath::vector3d(0, (lookahead * cell_size), 0);
    ret.egress = rpos + rmath::vector3d(cell_size,
                                        (lookahead * cell_size),
                                        0);
  } else if (rmath::radians::kPI == lane->orientation()) {
    /* We look in the NEGATIVE X direction for stygmergic configurations */
    ret.ingress = rpos + rmath::vector3d(-(lookahead * cell_size), 0, 0);
    ret.egress = rpos + rmath::vector3d(-(lookahead * cell_size),
                                        cell_size,
                                        0);
  } else if (rmath::radians::kTHREE_PI_OVER_TWO == lane->orientation()) {
    /* We look in the NEGATIVE Y direction for stygmergic configurations */
    ret.ingress = rpos + rmath::vector3d(0, -(lookahead * cell_size), 0);
    ret.egress = rpos + rmath::vector3d(-cell_size,
                                        -(lookahead * cell_size),
                                        0);
  } else {
    ER_FATAL_SENTINEL("Bad lane orientation '%s'",
                      rcppsw::to_string(lane->orientation()).c_str());
  }
  return ret;
} /* acq_positions_calc() */

NS_END(fsm, silicon);
