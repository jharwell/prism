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

#include "cosm/arena/base_arena_map.hpp"
#include "cosm/ds/operations/cell3D_empty.hpp"

#include "silicon/structure/operations/validate_placement.hpp"
#include "silicon/structure/subtarget.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
structure3D::structure3D(const config::structure3D_config* config,
                         const carena::base_arena_map* map,
                         size_t id)
    : grid3D_overlay(config->anchor,
                     config->bounding_box.dims,
                     config->bounding_box.resolution),
      ER_CLIENT_INIT("silicon.structure.structure3D"),
      mc_id(id),
      mc_block_unit_dim(std::min(map->blocks()[0]->dims3D().x(),
                                 map->blocks()[0]->dims3D().y())),
      mc_unit_dim_factor(unit_dim_factor_calc(map)),
      mc_arena_grid_res(map->grid_resolution()),
      mc_config(*config),
      m_cell_spec_map(cell_spec_map_calc()),
      m_subtargetso(subtargetso_calc()),
      m_subtargetsno(subtargetsno_calc()) {
  ER_ASSERT(rmath::radians::kZERO == mc_config.orientation ||
            rmath::radians::kPI_OVER_TWO == mc_config.orientation,
            "Bad structure orientation: '%s'",
            rcppsw::to_string(mc_config.orientation).c_str());

  for (uint i = 0; i < xdsize(); ++i) {
    for (uint j = 0; j < ydsize(); ++j) {
      for (uint k = 0; k < zdsize(); ++k) {
        rmath::vector3z c(i, j, k);
        auto& cell = access(c);
        cell.loc(c);
      } /* for(k..) */
    }   /* for(j..) */
  }     /* for(i..) */
}

structure3D::~structure3D(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool structure3D::block_placement_valid(const crepr::block3D_variant& block,
                                        const rmath::vector3z& loc,
                                        const rmath::radians& z_rotation) {
  /* common checks */
  auto& cell = access(loc);
  ER_CHECK(block_placement_cell_check(cell),
           "Host Cell@%s failed validation for block placement",
           rcppsw::to_string(cell.loc()).c_str());
  /*
   * @todo check if the embodiment for this block would overlap with any other
   * blocks already placed on the structure.
   */

  ER_CHECK(rmath::radians::kZERO == z_rotation ||
           rmath::radians::kPI_OVER_TWO == z_rotation,
           "Bad rotation %s: must be %s or %s",
           rcppsw::to_string(z_rotation).c_str(),
           rcppsw::to_string(rmath::radians::kZERO).c_str(),
           rcppsw::to_string(rmath::radians::kPI_OVER_TWO).c_str());

  /* checks specific to block type */
  return boost::apply_visitor(
      operations::validate_placement(this, loc, z_rotation), block);

error:
  return false;
} /* block_placement_valid() */

bool structure3D::block_placement_cell_check(const cds::cell3D& cell) const {
  ER_CHECK(!cell.fsm().state_has_block(),
           "Cell@%s already in ekST_HAS_BLOCK",
           rcppsw::to_string(cell.loc()).c_str());

  ER_CHECK(!cell.fsm().state_in_block_extent(),
           "Cell@%s already in ekST_BLOCK_EXTENT",
           rcppsw::to_string(cell.loc()).c_str());
  return true;

error:
  return false;
} /* block_placement_cell_check() */

bool structure3D::contains(const crepr::base_block3D* const query) const {
  return nullptr != cell_spec_retrieve(query->dpos3D());
} /* contains() */

bool structure3D::contains(const rmath::vector2d& loc,
                           bool include_virtual) const {
  if (include_virtual) {
    /*
     * The virtual cell border around the structure is the size of the block
     * unit dimension, so we use that for measuring.
     */
    auto xrange_exp = rmath::ranged(xrange().lb() - block_unit_dim(),
                                    xrange().ub() + block_unit_dim());
    auto yrange_exp = rmath::ranged(yrange().lb() - block_unit_dim(),
                                    yrange().ub() + block_unit_dim());
    return xrange_exp.contains(loc.x()) && yrange_exp.contains(loc.y());
  } else {
    return xrange().contains(loc.x()) && yrange().contains(loc.y());
  }
} /* contains() */

void structure3D::block_add(std::unique_ptr<crepr::base_block3D> block) {
  RCSW_UNUSED rtypes::type_uuid id = block->id();
  m_occupied_cells.push_back((block->dpos3D() - origind()) / mc_unit_dim_factor);
  m_placed.push_back(std::move(block));
  ++m_placed_since_reset;
  ER_INFO("Added block%d to structure (%zu total)",
          id.v(),
          m_placed.size());
} /* block_add() */

bool structure3D::is_complete(void) const {
  /*
   * @todo This is horribly simplistic and almost assuredly will need to be
   * updated substantially in the near future.
   */
  return n_placed_blocks() == n_total_blocks();
} /* is_complete() */

std::vector<rmath::vector3z> structure3D::spec_to_block_extents(
    const config::ramp_block_loc_spec* spec) const {
  std::vector<rmath::vector3z> ret;
  /*
   * The convention is that the loc field for ramp block specs always points to
   * the anchor point/host cell of the block, and that whatever the orientation
   * of the block is (X or Y), it extends in the POSITIVE direction for that
   * orientation.
   */
  if (spec->z_rotation == rmath::radians::kZERO) {
    for (uint m = 1; m < kRAMP_BLOCK_EXTENT_SIZE; ++m) {
      ret.push_back({spec->loc.x() + m, spec->loc.y(), spec->loc.z()});
    } /* for(m..) */
  } else if (spec->z_rotation == rmath::radians::kPI_OVER_TWO) {
    for (uint m = 1; m < kRAMP_BLOCK_EXTENT_SIZE; ++m) {
      ret.push_back({spec->loc.x(), spec->loc.y() + m, spec->loc.z()});
    } /* for(m..) */
  } else {
    ER_FATAL_SENTINEL("Bad rotation '%s' in spec",
                      rcppsw::to_string(spec->z_rotation).c_str());
  }
  return ret;
} /* spec_to_block_extents() */

structure3D::cell_spec structure3D::cell_spec_calc(
    const rmath::vector3z& coord) const {
  ER_DEBUG("Query spec for cell@%s", rcppsw::to_string(coord).c_str());
  /*
   * Direct key comparison for host cells. This is the default, but I explicitly
   * define it here to clearly differentiate it from searching for blocks which
   * match the specified location based on extents.
   */
  auto host_pred = [&](const auto& pair) { return pair.first == coord; };

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
    auto sum =
        std::accumulate(std::begin(extents),
                        std::end(extents),
                        std::string(),
                        [&](const std::string& a, const rmath::vector3z& l) {
                          return a + rcppsw::to_string(l) + ",";
                        });

    ER_TRACE("Checking block extent cells %s (%zu)",
             sum.c_str(),
             extents.size());
    /*
     * Check all extents for the current block to
     * see if they match the location we were
     * passed.
     */
    for (auto& e : extents) {
      if (e == coord) {
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
            rcppsw::to_string(coord).c_str());

  if (mc_config.cube_blocks.end() != cube_it) {
    return {cfsm::cell3D_state::ekST_HAS_BLOCK,
            crepr::block_type::ekCUBE,
            rmath::radians::kZERO,
            1};
  } else if (mc_config.ramp_blocks.end() != ramp_host_it) {
    return {cfsm::cell3D_state::ekST_HAS_BLOCK,
            crepr::block_type::ekRAMP,
            ramp_host_it->second.z_rotation,
            kRAMP_BLOCK_EXTENT_SIZE};
  } else if (mc_config.ramp_blocks.end() != ramp_extent_it) {
    return {cfsm::cell3D_state::ekST_BLOCK_EXTENT,
            crepr::block_type::ekNONE,
            rmath::radians::kZERO,
            0};
  } else {
    return {cfsm::cell3D_state::ekST_EMPTY,
            crepr::block_type::ekNONE,
            rmath::radians::kZERO,
            0};
  }
} /* cell_spec_calc() */

rmath::vector3d structure3D::cell_loc_abs(const cds::cell3D& cell) const {
  return originr() + rmath::zvec2dvec(cell.loc()) * mc_unit_dim_factor *
                         mc_arena_grid_res.v();
} /* cell_loc_abs() */

size_t structure3D::unit_dim_factor_calc(const carena::base_arena_map* map) const {
  ER_ASSERT(std::fmod(mc_block_unit_dim, map->grid_resolution().v()) <=
            std::numeric_limits<double>::epsilon(),
            "Block unit dimension (%f) not a multiple of arena grid resolution (%f)",
            mc_block_unit_dim,
            map->grid_resolution().v());
  return static_cast<size_t>(mc_block_unit_dim / map->grid_resolution().v());
} /* unit_dim_factor_calc() */

structure3D::subtarget_vectoro structure3D::subtargetso_calc(void) const {
  subtarget_vectoro ret;
  if (rmath::radians::kZERO == mc_config.orientation) {
    for (size_t j = 0; j < ydsize() / 2; ++j) {
      ret.push_back(std::make_unique<subtarget>(this, j));
    } /* for(j..) */
  } else {
    for (size_t i = 0; i < xdsize() / 2; ++i) {
      ret.push_back(std::make_unique<subtarget>(this, i));
    } /* for(i..) */
  }
  return ret;
} /* subtargetso_calc() */

structure3D::subtarget_vectorno structure3D::subtargetsno_calc(void) const {
  subtarget_vectorno ret;
  std::transform(m_subtargetso.begin(),
                 m_subtargetso.end(),
                 std::back_inserter(ret),
                 [&](const auto& s) { return s.get(); });
  return ret;
} /* subtargetsno_calc() */

structure3D::cell_spec_map_type structure3D::cell_spec_map_calc(void) {
  cell_spec_map_type ret;
  ER_DEBUG("Build cell spec map for structure%d", mc_id.v());
  for (size_t i = 0; i < xdsize(); ++i) {
    for (size_t j = 0; j < ydsize(); ++j) {
      for (size_t k = 0; k < zdsize(); ++k) {
        rmath::vector3z c(i, j, k);
        ret.insert({c, cell_spec_calc(c)});
      } /* for(k..) */
    }   /* for(j..) */
  }     /* for(i..) */
  ER_DEBUG("Cell spec map for structure%d complete", mc_id.v());
  return ret;
} /* cell_spec_map_calc() */

subtarget* structure3D::cell_subtarget(const cds::cell3D& cell) {
  if (rmath::radians::kZERO == mc_config.orientation) {
    return m_subtargetsno[cell.loc().y() / 2];
  } else {
    return m_subtargetsno[cell.loc().x() / 2];
  } /* for(i..) */
} /* cell_subtarget() */

const structure3D::cell_spec* structure3D::cell_spec_retrieve(
    const rmath::vector3z& coord) const {
  auto it = m_cell_spec_map.find(coord);
  if (m_cell_spec_map.end() != it) {
    return &it->second;
  }
  return nullptr;
}

void structure3D::reset(void) {
  m_placed.clear();
  m_placement_id = 0;
  reset_metrics();

  for (uint i = 0; i < xdsize(); ++i) {
    for (uint j = 0; j < ydsize(); ++j) {
      for (uint k = 0; k < zdsize(); ++k) {
        rmath::vector3z coord(i, j, k);
        cdops::cell3D_empty op(coord);
        op.visit(access(coord));
      } /* for(k..) */
    }   /* for(j..) */
  }     /* for(i..) */
} /* reset() */

void structure3D::reset_metrics(void) {
  m_placed_since_reset = 0;
  for (auto *t : subtargets()) {
    t->reset_metrics();
  } /* for(*t..) */
} /* reset_metrics() */

NS_END(structure, silicon);
