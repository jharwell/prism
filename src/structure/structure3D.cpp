/**
 * \file structure3D.cpp
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
#include "silicon/structure/structure3D.hpp"

#include <algorithm>
#include <boost/variant/static_visitor.hpp>

#include "silicon/structure/operations/validate_placement.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
structure3D::structure3D(const config::structure3D_config* config)
    : grid3D(config->bounding_box),
      ER_CLIENT_INIT("silicon.structure.structure3D"),
      mc_config(*config) {
    for (uint i = 0; i < mc_config.bounding_box.x(); ++i) {
      for (uint j = 0; j < mc_config.bounding_box.y(); ++j) {
        for (uint k = 0; k < mc_config.bounding_box.z(); ++k) {
          rmath::vector3u c(i, j, k);
          auto& cell = access(c);
          cell.loc(c);
        } /* for(k..) */
      } /* for(j..) */
    } /* for(i..) */
  }

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool structure3D::block_placement_valid(const crepr::block3D_variant& block,
                                        const rmath::vector3u& loc,
                                        const rmath::radians& z_rotation) {
  /* common checks */
  auto& cell = access(loc);
  ER_CHECK(block_placement_cell_check(cell),
           "Host Cell@%s failed validation for block placement",
           cell.loc().to_str().c_str());

  ER_CHECK(rmath::radians::kZERO == z_rotation ||
           rmath::radians::kPI_OVER_TWO == z_rotation,
           "Bad rotation %s: must be %s or %s",
           z_rotation.to_str().c_str(),
           rmath::radians::kZERO.to_str().c_str(),
           rmath::radians::kPI_OVER_TWO.to_str().c_str());

  /* checks specific to block type */
  return boost::apply_visitor(operations::validate_placement(this,
                                                             loc,
                                                             z_rotation),
                              block);

error:
  return false;
} /* block_placement_valid() */

bool structure3D::block_placement_cell_check(const cds::cell3D& cell) const {
  ER_CHECK(!cell.fsm().state_has_block(),
           "Cell@%s already in ekST_HAS_BLOCK",
           cell.loc().to_str().c_str());

  ER_CHECK(!cell.fsm().state_in_block_extent(),
           "Cell@%s already in ekST_BLOCK_EXTENT",
           cell.loc().to_str().c_str());
  return true;

error:
  return false;
} /* block_placement_cell_check() */

bool structure3D::contains(const crepr::base_block3D* const query) const {
  auto it = std::find_if(m_placed.begin(),
                         m_placed.end(),
                         [&] (const auto* block) { return block->idcmp(*query); });
  return m_placed.end() != it;
} /* contains() */

void structure3D::block_add(const crepr::base_block3D* block) {
  m_placed.push_back(block);
} /* block_add() */

bool structure3D::is_complete(void) const {
  /*
   * @todo This is horribly simplistic and almost assuredly will need to be
   * updated substantially in the near future.
   */
  return m_placed.size() == mc_config.cube_blocks.size() +
      mc_config.ramp_blocks.size();
} /* is_complete() */

std::vector<rmath::vector3u> structure3D::spec_to_block_extents(
    const config::ramp_block_loc_spec* spec) const {

  /*
   * The convention is that the loc field for ramp block specs always points to
   * the anchor point/host cell of the block, and that whatever the orienttion
   * of the block is (X or Y), it extends in the POSITIVE direction for that
   * orientation.
   */
  if (spec->z_rotation == rmath::radians::kZERO) {
    rmath::vector3u xplus1(spec->loc.x() + 1, spec->loc.y(), spec->loc.z());
    return {xplus1};
  } else {
    rmath::vector3u yplus1(spec->loc.x(), spec->loc.y() + 1, spec->loc.z());
    return {yplus1};
  }
} /* spec_to_block_extents() */

structure3D::cell_final_spec structure3D::cell_spec(
    const rmath::vector3u& loc) const {
  /*
   * Direct key comparison for host cells. This is the default, but I explicitly
   * define it here to clearly differentiate it from searching for blocks which
   * match the specified location based on extents.
   */
  auto host_pred = [&](const auto& pair) {
    return pair.first == loc;
  };

  /* easy case: cubes are 1x1x1, so they have no extents */
  auto cube_it = std::find_if(mc_config.cube_blocks.begin(),
                              mc_config.cube_blocks.end(),
                              host_pred);

  /*
   * Harder case: for ramps we need to compare the host cell location AND
   * figure out if the passed location is part of a block extent, in order to
   * return the correct cell target state.
   */
  auto extent_pred = [&](const auto& pair) {
    auto extents = spec_to_block_extents(&pair.second);

    /*
     * Check all extents for the current block to
     * see if they match the location we were
     * passed.
     */
    for (auto &e : extents) {
      if (e == pair.second.loc) {
        return true;
      }
    } /* for(&e..) */
    return false;
  };

  auto ramp_host_it = std::find_if(mc_config.ramp_blocks.begin(),
                                   mc_config.ramp_blocks.end(),
                                   host_pred);
  auto ramp_extent_it = std::find_if(mc_config.ramp_blocks.begin(),
                                   mc_config.ramp_blocks.end(),
                                   extent_pred);

  uint count = (mc_config.ramp_blocks.end() != ramp_host_it) +
               (mc_config.ramp_blocks.end() != ramp_extent_it) +
               (mc_config.cube_blocks.end() != cube_it);
  ER_ASSERT(count <= 1,
            "Cell@%s config error: found in more than block spec map",
            loc.to_str().c_str());

  if (mc_config.cube_blocks.end() != cube_it) {
    return {cfsm::cell3D_state::ekST_HAS_BLOCK,
          crepr::block_type::ekCUBE,
          rmath::radians::kZERO};
  } else if (mc_config.ramp_blocks.end() != ramp_host_it) {
    return {cfsm::cell3D_state::ekST_HAS_BLOCK,
          crepr::block_type::ekRAMP,
          ramp_host_it->second.z_rotation};
  } else if (mc_config.ramp_blocks.end() != ramp_extent_it) {
    return {cfsm::cell3D_state::ekST_BLOCK_EXTENT,
          crepr::block_type::ekNONE,
          rmath::radians::kZERO};
  } else {
    return {cfsm::cell3D_state::ekST_EMPTY,
          crepr::block_type::ekNONE,
          rmath::radians::kZERO};
  }
} /* cell_spec() */

NS_END(structure, silicon);
