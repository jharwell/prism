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

#include "cosm/subsystem/sensing_subsystemQ3D.hpp"
#include "cosm/ds/cell3D.hpp"

#include "silicon/repr/construction_lane.hpp"
#include "silicon/controller/perception/builder_perception_subsystem.hpp"

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
  rmath::vector3z ingress_los_rel;
  rmath::vector3z egress_los_rel;

  auto* los = mc_perception->los();

  /* We were called from outside of the structure */
  if (nullptr == los) {
    return stygmergic_configuration::ekNONE;
  }

  /*
   * \todo Right now this assumes cube blocks, and is only correct for
   * such. This will be fixed once the most basic construction has been
   * validated.
   */
  if (rmath::radians::kZERO == lane->orientation()) {
    ingress_los_rel = rmath::vector3z(0, 0, 0);
    egress_los_rel = rmath::vector3z::Y;
  } else {
    ER_FATAL_SENTINEL("Bad lane orientation '%s'",
                      rcppsw::to_string(lane->orientation()).c_str());
  }

  auto& ingress_cell = los->access(ingress_los_rel);
  auto& egress_cell = los->access(egress_los_rel);

  bool ingress_has_block = ingress_cell.state_has_block();
  bool egress_has_block = egress_cell.state_has_block();
  ER_DEBUG("robot position: %s/%s LOS origin: %s",
           rcppsw::to_string(mc_sensing->rpos3D()).c_str(),
           rcppsw::to_string(mc_sensing->dpos3D()).c_str(),
           rcppsw::to_string(los->abs_ll()).c_str());
  ER_DEBUG("ingress_los_rel: %s egress_los_rel: %s",
           rcppsw::to_string(ingress_los_rel).c_str(),
           rcppsw::to_string(egress_los_rel).c_str());

  /* stygmergic configurations for partially complete construction lanes */
  if (ingress_has_block && egress_has_block) {
    ER_INFO("LANE_FILLED configuration encountered");
    return stygmergic_configuration::ekLANE_FILLED;
  } else if (ingress_has_block && !egress_has_block) {
    ER_INFO("LANE_GAP_EGRESS configuration encountered");
    return stygmergic_configuration::ekLANE_GAP_EGRESS;
  } else if (!ingress_has_block && egress_has_block) {
    ER_INFO("LANE_GAP_INGRESS configuration encountered");
    return stygmergic_configuration::ekLANE_GAP_INGRESS;
  } else if (0 == ingress_cell.loc().x() && 0 == egress_cell.loc().x()) {
    ER_INFO("LANE_EMPTY configuration encountered");
    return stygmergic_configuration::ekLANE_EMPTY;
  }
  return stygmergic_configuration::ekNONE;
} /* operator()() */

NS_END(fsm, silicon);
