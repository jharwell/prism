/**
 * \file cubic_spacefill.cpp
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
#include "prism/fsm/calculators/fs_acq/cubic_spacefill.hpp"

#include "rcppsw/math/radians.hpp"

#include "cosm/subsystem/sensing_subsystemQ3D.hpp"
#include "cosm/repr/base_block3D.hpp"

#include "prism/controller/perception/builder_perception_subsystem.hpp"
#include "prism/repr/construction_lane.hpp"
#include "prism/repr/builder_los.hpp"
#include "prism/properties/algorithm.hpp"
#include "prism/gmt/utils.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, fsm, calculators, fs_acq);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
cubic_spacefill::cubic_spacefill(
    const csubsystem::sensing_subsystemQ3D* sensing,
    const pcperception::builder_perception_subsystem* perception)
    : ER_CLIENT_INIT("prism.fsm.cubic_spacefill"),
      base_strategy(sensing, perception) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
prepr::fs_acq_result
cubic_spacefill::operator()(const prepr::construction_lane* lane) const {
  const auto* los = perception()->los();

  prepr::fs_acq_result result;
  if (nullptr == los) {
    return result;
  }

  ER_TRACE("Robot position: %s/%s",
           rcppsw::to_string(sensing()->rpos3D()).c_str(),
           rcppsw::to_string(sensing()->dpos3D()).c_str());

  /*
   * \todo Right now this assumes cube blocks, and is only correct for
   * such. This will be fixed once the most basic construction has been
   * validated.
   */

  for (size_t lookahead = 1; lookahead <= ppalgorithm::kCT_FS_LOOKAHEAD_MAX_CELLS; ++lookahead) {
    result = acq_result_calc(lane, los, lookahead);

    /*
     * Give priority to configurations encountered closer to the robot: as soon
     * as we find one, don't look any further out.
     */
    if (prepr::fs_configuration::ekNONE != result.configuration) {
      break;
    }
  } /* for(lookahead..) */
  return result;
} /* operator()() */

prepr::fs_acq_result
cubic_spacefill::acq_result_calc(const prepr::construction_lane* lane,
                            const prepr::builder_los* los,
                            size_t lookahead) const {
  prepr::fs_acq_result ret;

  const auto* ct = perception()->nearest_ct();
  auto robot_ct_cell = ct->to_rcoord(sensing()->rpos3D());

  ER_CONDW(boost::none == los->find(robot_ct_cell.offset()),
            "Robot CT cell not in LOS?");

  auto rpos = sensing()->rpos3D();
  auto dpos = sensing()->dpos3D();

  ret.positions = acq_positions_calc(rpos, lane, lookahead);

  auto ingress_arena_cell = rmath::dvec2zvec(ret.positions.ingress,
                                             perception()->arena_resolution().v());
  auto egress_arena_cell = rmath::dvec2zvec(ret.positions.egress,
                                            perception()->arena_resolution().v());
  auto ingress_ct_cell = ct->to_rcoord(ret.positions.ingress);
  auto egress_ct_cell = ct->to_rcoord(ret.positions.egress);


  ER_TRACE("Robot arena cell=%s,ingress arena cell=%s,egress arena cell=%s",
           rcppsw::to_string(dpos).c_str(),
           rcppsw::to_string(ingress_arena_cell).c_str(),
           rcppsw::to_string(egress_arena_cell).c_str());

  ER_TRACE("Robot CT cell=%s,ingress CT cell=%s,egress CT cell=%s",
           rcppsw::to_string(robot_ct_cell).c_str(),
           rcppsw::to_string(ingress_ct_cell).c_str(),
           rcppsw::to_string(egress_ct_cell).c_str());

  /*
   * This is not an error because when we first enter the virtual shell of the
   * structure and get a LOS, our "lookahead" might exceed the size of the
   * LOS. In general once we are on the structure proper this should never
   * happen.
   */
  if (auto ingress_vd = los->find(ingress_ct_cell.offset())) {
    ret.specs.ingress = los->access(*ingress_vd);
  } else {
    ER_WARN("LOS does not contain ingress lane frontier cell candidate %s",
            rcppsw::to_string(ingress_ct_cell).c_str());
  }

  if (auto egress_vd = los->find(egress_ct_cell.offset())) {
    ret.specs.egress = los->access(*egress_vd);
  } else {
    ER_WARN("LOS does not contain egress lane frontier cell candidate %s",
            rcppsw::to_string(egress_ct_cell).c_str());
  }

  ret.lookahead = lookahead;
  ret.configuration = configuration_calc(ret, lane, los);
  return ret;
} /* acq_result_calc() */

prepr::fs_configuration
cubic_spacefill::configuration_calc(const prepr::fs_acq_result& result,
                               const prepr::construction_lane* lane,
                               const prepr::builder_los* los) const {
  if (nullptr != result.specs.ingress && nullptr != result.specs.egress) {
    bool ingress_has_block = nullptr != result.specs.ingress->block;
    bool egress_has_block = nullptr != result.specs.egress->block;

    if (ingress_has_block && egress_has_block) {
      ER_INFO("LANE_FILLED encountered: ingress=%s,egress=%s,lookahead=%zu",
              rcppsw::to_string(result.specs.ingress->coord).c_str(),
              rcppsw::to_string(result.specs.egress->coord).c_str(),
              result.lookahead);
      return prepr::fs_configuration::ekLANE_FILLED;
    } else if (ingress_has_block && !egress_has_block) {
      ER_INFO("LANE_GAP_EGRESS encountered: ingress=%s,egress=%s,lookahead=%zu",
              rcppsw::to_string(result.specs.ingress->coord).c_str(),
              rcppsw::to_string(result.specs.egress->coord).c_str(),
              result.lookahead);
      return prepr::fs_configuration::ekLANE_GAP_EGRESS;
    } else if (!ingress_has_block && egress_has_block) {
      ER_INFO("LANE_GAP_INGRESS encountered: ingress=%s,egress=%s,lookahead=%zu",
              rcppsw::to_string(result.specs.ingress->coord).c_str(),
              rcppsw::to_string(result.specs.egress->coord).c_str(),
              result.lookahead);
      return prepr::fs_configuration::ekLANE_GAP_INGRESS;
    }
  } else {
    /* possibly empty lane encountered--need to check to see for sure */
    if (acq_empty_lane(lane, los)) {
          ER_INFO("LANE_EMPTY encountered: ingress=%s,egress=%s,lookahead=%zu",
              rcppsw::to_string(result.positions.ingress).c_str(),
              rcppsw::to_string(result.positions.egress).c_str(),
              result.lookahead);
      return prepr::fs_configuration::ekLANE_EMPTY;
    }
  }

  return prepr::fs_configuration::ekNONE;
} /* configuration_calc() */

bool cubic_spacefill::acq_empty_lane(const prepr::construction_lane* lane,
                                    const prepr::builder_los* los) const {
  const auto* ct = perception()->nearest_ct();
  ER_ASSERT(pgmt::orientation_valid(lane->orientation()),
            "Bad orientation: '%s'",
            rcppsw::to_string(lane->orientation()).c_str());

  /*
   * The origin of the LOS is always in the LL corner, regardless of lane
   * orientation, which simplifies calculations of empty lane acquisition a
   * lot. The returned spans of the LOS are specified relative to the grid
   * origin (structure real origin), so we account for that here to get spans
   * relative to the arena origin.
   */
  auto xspan = los->xrspan().recenter(sensing()->rpos3D().x());
  auto yspan = los->yrspan().recenter(sensing()->rpos3D().y());

  if (rmath::radians::kZERO == lane->orientation()) {
    return xspan.ub() >= ct->roriginr().x() + ct->bbr(false).x();
  } else if (rmath::radians::kPI_OVER_TWO == lane->orientation()) {
    return yspan.ub() >= ct->roriginr().y() + ct->bbr(false).y();
  } else if (rmath::radians::kPI == lane->orientation()) {
    return xspan.lb() <= ct->roriginr().x();
  } else if (rmath::radians::kTHREE_PI_OVER_TWO == lane->orientation()) {
    return yspan.lb() <= ct->roriginr().y();
  }
  return false;
} /* acq_empty_lane() */

prepr::fs_acq_positions cubic_spacefill::acq_positions_calc(
    const rmath::vector3d& rpos,
    const prepr::construction_lane* lane,
    size_t lookahead) const {
  prepr::fs_acq_positions ret;

  ER_ASSERT(pgmt::orientation_valid(lane->orientation()),
            "Bad orientation: '%s'",
            rcppsw::to_string(lane->orientation()).c_str());

  const auto* ct = perception()->nearest_ct();
  auto cell_size = ct->block_unit_dim().v();

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
  }
  return ret;
} /* acq_positions_calc() */

NS_END(fs_acq, calculators, fsm, prism);
