/**
 * \file block_placement_validate.cpp
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
#include "prism/gmt/operations/block_placement_validate.hpp"

#include "cosm/repr/cube_block3D.hpp"
#include "cosm/repr/ramp_block3D.hpp"

#include "prism/gmt/spc_gmt.hpp"
#include "prism/gmt/utils.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
bool block_placement_validate::operator()(const crepr::cube_block3D* block) const {
  ER_CHECK(validate_common(),
           "Common validation for cube block%d with intent %s failed",
           block->id().v(),
           rcppsw::to_string(mc_intent).c_str());

  /* @todo check if structure invariants are violated by adding this block */
  return true;

error:
  ER_ERR("Cube block%d placement with intent %s failed validation",
         block->id().v(),
         rcppsw::to_string(mc_intent).c_str());
  return false;
} /* operator() */

bool block_placement_validate::operator()(const crepr::ramp_block3D* block) const {
  ER_CHECK(validate_common(),
           "Common validation for ramp block%d with intent %s failed",
           block->id().v(),
           rcppsw::to_string(mc_intent).c_str());

  if (rmath::radians::kZERO == mc_intent.z_rot()) {
    /* \todo check if structure invariants are violated by adding this block */
  } else if (rmath::radians::kPI == mc_intent.z_rot()) {
    /* \todo check if structure invariants are violated by adding this block */
  } else {
    ER_FATAL_SENTINEL("Bad orientation: %s",
                      rcppsw::to_string(mc_intent.z_rot()).c_str());
  }
error:
  ER_ERR("Ramp block%d placement with intent %s failed validation",
         block->id().v(),
         rcppsw::to_string(mc_intent).c_str());
  return false;
} /* operator() */

bool block_placement_validate::validate_common(void) const {
  /* all accesses into 3D array must be relative to virtual origin */
  ER_ASSERT(pgrepr::ct_coord::relativity::ekVORIGIN ==
            mc_intent.site().relative_to(),
            "Placement coordinates not relative to virtual origin");

  auto vcoord = mc_intent.site();
  const auto* spec = mc_target->spec_retrieve(vcoord);

  /* Site must be a block anchor point */
  ER_CHECK(nullptr != spec,
           "Cell@%s not a block anchor point",
           rcppsw::to_string(vcoord).c_str());

  /* Site can't already have block */
  ER_CHECK(nullptr == spec->block,
           "Cell@%s already is block anchor point/contains block%d",
           rcppsw::to_string(spec->coord).c_str(),
           spec->block->id().v());

  /* check placement intent rotation valid */
  ER_CHECK(orientation_valid(mc_intent.z_rot(), kZ_ROT_TOL),
           "Bad rotation %s: must be 0,pi/2,pi,3pi/2",
           rcppsw::to_string(mc_intent.z_rot()).c_str());

  /* check placement intent rotation matches block rotation from spec */
  ER_CHECK(mc_intent.z_rot() == spec->z_rot,
           "Placement intent rotation %s != spec rotation %s",
           rcppsw::to_string(mc_intent).c_str(),
           rcppsw::to_string(spec->z_rot).c_str());
  /*
   * \todo check if the embodiment for this block would overlap with any other
   * blocks already placed on the structure.
   */

  return true;

error:
  return false;
} /* validate_common() */

NS_END(operations, gmt, prism);
