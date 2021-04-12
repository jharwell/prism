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

#include "silicon/structure/calculators/ramp_block_extent.hpp"
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
    : grid3D_overlay(config->anchor -
                         /*
                      * Subtract padding for the two layers of virtual cells
                      * surrounding the structure so the origin of the real
                      * structure is as configured in the input file.
                      */
                         rmath::vector3d(
                             config->bounding_box.resolution.v() * vshell_sized(),
                             config->bounding_box.resolution.v() * vshell_sized(),
                             0.0),
                     config->bounding_box.dims +
                         /*
                      * Add padding for the two layers of virtual cells
                      * surrounding the structure (* 2 because padding is added
                      * on both X,Y sides).
                      */
                         rmath::vector3d(config->bounding_box.resolution.v() *
                                             vshell_sized() * 2,
                                         config->bounding_box.resolution.v() *
                                             vshell_sized() * 2,
                                         0.0),
                     config->bounding_box.resolution,
                     map->grid_resolution()),
      ER_CLIENT_INIT("silicon.structure.structure3D"),
      mc_id(id),
      mc_block_unit_dim(std::min(map->blocks()[0]->rdim2D().x(),
                                 map->blocks()[0]->rdim2D().y())),
      mc_unit_dim_factor(unit_dim_factor_calc(map)),
      mc_arena_grid_res(map->grid_resolution()),
      mc_config(*config) {
  ER_ASSERT(orientation_valid(orientation()),
            "Bad structure orientation: '%s'",
            rcppsw::to_string(orientation()).c_str());
  ER_ASSERT(std::fabs(config->bounding_box.resolution.v() - mc_block_unit_dim) <=
                std::numeric_limits<double>::epsilon(),
            "Resolution of 3D grid does not match block unit dimension (%s != "
            "%f)",
            rcppsw::to_string(config->bounding_box.resolution).c_str(),
            mc_block_unit_dim);
  ER_INFO("Structure%s: vorigin=%s/%s, rorigin=%s/%s",
          rcppsw::to_string(mc_id).c_str(),
          rcppsw::to_string(voriginr()).c_str(),
          rcppsw::to_string(vorigind()).c_str(),
          rcppsw::to_string(roriginr()).c_str(),
          rcppsw::to_string(rorigind()).c_str());

  ER_INFO("Structure%s: dims=%s/%s, rxrange=%s,ryrange=%s vxrange=%s,vyrange=%s",
          rcppsw::to_string(mc_id).c_str(),
          rcppsw::to_string(dimsd()).c_str(),
          rcppsw::to_string(dimsr()).c_str(),
          rcppsw::to_string(xranger(false)).c_str(),
          rcppsw::to_string(yranger(false)).c_str(),
          rcppsw::to_string(xranger(true)).c_str(),
          rcppsw::to_string(yranger(true)).c_str());

  for (size_t i = 0; i < xdsize(); ++i) {
    for (size_t j = 0; j < ydsize(); ++j) {
      for (size_t k = 0; k < zdsize(); ++k) {
        rmath::vector3z c(i, j, k);
        auto& cell = access(c);
        cell.loc(c);
      } /* for(k..) */
    } /* for(j..) */
  } /* for(i..) */

  /* now that cell locations are populated, finish structure initialization */
  m_cell_spec_map = cell_spec_map_init();
  m_subtargetso = subtargetso_init();
  m_subtargetsno = subtargetsno_init();
}

structure3D::~structure3D(void) = default;

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/
structure3D::cell_spec_map_type structure3D::cell_spec_map_init(void) {
  cell_spec_map_type ret;
  size_t shell = vshell_sized();
  ER_DEBUG(
      "Build cell spec map for structure%d: shell_size=%zu", mc_id.v(), shell);
  for (size_t i = shell; i < xdsize() - shell; ++i) {
    for (size_t j = shell; j < ydsize() - shell; ++j) {
      for (size_t k = 0; k < zdsize(); ++k) {
        /*
         * We have to subtract the shell size to make the coordinates relative
         * to the ACTUAL origin of the structure--NOT the origin of the
         * structure shell.
         */
        rmath::vector3z c(i - shell, j - shell, k);
        ret.insert({ c, cell_spec_calc(c) });
      } /* for(k..) */
    } /* for(j..) */
  } /* for(i..) */
  ER_DEBUG("Cell spec map for structure%d complete", mc_id.v());
  return ret;
} /* cell_spec_map_init() */

structure3D::cell_spec
structure3D::cell_spec_calc(const rmath::vector3z& coord) const {
  ER_DEBUG("Query spec for cell@%s", rcppsw::to_string(coord).c_str());
  /*
   * Direct key comparison for host cells. This is the default, but I explicitly
   * define it here to clearly differentiate it from searching for blocks which
   * match the specified location based on extents.
   */
  auto host_pred = [&](const auto& pair) { return pair.first == coord; };

  /* easy case: cubes are 1x1x1, so they have no extents */
  auto cube_it = std::find_if(
      mc_config.cube_blocks.begin(), mc_config.cube_blocks.end(), host_pred);

  /*
   * Harder case: for ramps we need to compare the host cell location AND
   * figure out if the passed location is part of a block extent, in order to
   * return the correct cell target state.
   */
  auto extent_pred = [&](const auto& pair) {
    auto extents = calculators::ramp_block_extent()(&pair.second);
    auto sum =
        std::accumulate(std::begin(extents),
                        std::end(extents),
                        std::string(),
                        [&](const std::string& a, const rmath::vector3z& l) {
                          return a + rcppsw::to_string(l) + ",";
                        });

    ER_TRACE("Checking block extent cells %s (%zu)", sum.c_str(), extents.size());
    /*
     * Check all extents for the current block to see if they match the location
     * we were passed.
     */
    return extents.end() != std::find(extents.begin(), extents.end(), coord);
  };

  auto ramp_host_it = std::find_if(
      mc_config.ramp_blocks.begin(), mc_config.ramp_blocks.end(), host_pred);
  auto ramp_extent_it = std::find_if(
      mc_config.ramp_blocks.begin(), mc_config.ramp_blocks.end(), extent_pred);

  uint count = (mc_config.ramp_blocks.end() != ramp_host_it) +
               (mc_config.ramp_blocks.end() != ramp_extent_it) +
               (mc_config.cube_blocks.end() != cube_it);
  ER_ASSERT(count <= 1,
            "Cell@%s config error: found in more than one block spec map",
            rcppsw::to_string(coord).c_str());

  if (mc_config.cube_blocks.end() != cube_it) {
    return { cfsm::cell3D_state::ekST_HAS_BLOCK,
             crepr::block_type::ekCUBE,
             rmath::radians::kZERO,
             1 };
  } else if (mc_config.ramp_blocks.end() != ramp_host_it) {
    return { cfsm::cell3D_state::ekST_HAS_BLOCK,
             crepr::block_type::ekRAMP,
             ramp_host_it->second.z_rotation,
             calculators::ramp_block_extent::kEXTENT_SIZE };
  } else if (mc_config.ramp_blocks.end() != ramp_extent_it) {
    return { cfsm::cell3D_state::ekST_BLOCK_EXTENT,
             crepr::block_type::ekNONE,
             rmath::radians::kZERO,
             0 };
  } else {
    return { cfsm::cell3D_state::ekST_EMPTY,
             crepr::block_type::ekNONE,
             rmath::radians::kZERO,
             0 };
  }
} /* cell_spec_calc() */

structure3D::subtarget_vectoro structure3D::subtargetso_init(void) const {
  subtarget_vectoro ret;
  if (rmath::radians::kZERO == orientation() ||
      rmath::radians::kPI == orientation()) {
    for (size_t j = 0; j < ydsize() / kSUBTARGET_CELL_WIDTH - vshell_sized();
         ++j) {
      ER_INFO("Calculating subtarget %zu along Y slice axis", j);
      ret.push_back(std::make_unique<subtarget>(this, j));
    } /* for(j..) */
  } else if (rmath::radians::kPI_OVER_TWO == orientation() ||
             rmath::radians::kTHREE_PI_OVER_TWO == orientation()) {
    for (size_t i = 0; i < xdsize() / kSUBTARGET_CELL_WIDTH - vshell_sized();
         ++i) {
      ER_INFO("Calculating subtarget %zu along X slice axis", i);
      ret.push_back(std::make_unique<subtarget>(this, i));
    } /* for(i..) */
  } else {
    ER_FATAL_SENTINEL("Bad orientation : %s",
                      rcppsw::to_string(orientation()).c_str());
  }
  return ret;
} /* subtargetso_init() */

structure3D::subtarget_vectorno structure3D::subtargetsno_init(void) const {
  subtarget_vectorno ret;
  std::transform(m_subtargetso.begin(),
                 m_subtargetso.end(),
                 std::back_inserter(ret),
                 [&](const auto& s) { return s.get(); });
  return ret;
} /* subtargetsno_init() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool structure3D::block_placement_valid(const crepr::block3D_variantro& block,
                                        const ct_coord& coord,
                                        const rmath::radians& z_rotation) {
  auto validator = operations::validate_placement(this, coord, z_rotation);
  return boost::apply_visitor(validator, block);
} /* block_placement_valid() */

bool structure3D::spec_exists(const crepr::base_block3D* const query) const {
  return nullptr !=
         cell_spec_retrieve({ query->danchor3D(), coord_relativity::ekVORIGIN });
} /* spec_exists() */

bool structure3D::contains(const rmath::vector2d& loc,
                           bool include_virtual) const {
  return xranger(include_virtual).contains(loc.x()) &&
         yranger(include_virtual).contains(loc.y());
} /* contains() */

void structure3D::block_add(std::unique_ptr<crepr::base_block3D> block) {
  RCSW_UNUSED rtypes::type_uuid id = block->id();
  m_occupied_cells.push_back((block->danchor3D() - origind()) /
                             mc_unit_dim_factor);
  m_placed.push_back(std::move(block));
  ++m_placed_since_reset;
  ER_INFO("Added block%d to structure (%zu total)", id.v(), m_placed.size());
} /* block_add() */

const structure3D::cell_spec*
structure3D::cell_spec_retrieve(const ct_coord& coord) const {
  /*
   * If the coordinates are relative to the virtual origin, we have to translate
   * them to be relative to the REAL origin, as only cells that are ACTUALLY
   * part of the structure's bounding box will have had specs calculated for
   * them.
   */
  auto rcoord = to_rcoord(coord, this);
  auto it = m_cell_spec_map.find(rcoord);
  if (m_cell_spec_map.end() != it) {
    return &it->second;
  }
  return nullptr;
} /* cell_spec_retrieve() */

rmath::vector3d structure3D::cell_loc_abs(const cds::cell3D& cell) const {
  return voriginr() +
         rmath::zvec2dvec(cell.loc()) * unit_dim_factor() * mc_arena_grid_res.v();
} /* cell_loc_abs() */

size_t
structure3D::unit_dim_factor_calc(const carena::base_arena_map* map) const {
  ER_ASSERT(std::fmod(mc_block_unit_dim, map->grid_resolution().v()) <=
                std::numeric_limits<double>::epsilon(),
            "Block unit dimension (%f) not a multiple of arena grid resolution "
            "(%f)",
            mc_block_unit_dim,
            map->grid_resolution().v());
  return static_cast<size_t>(mc_block_unit_dim / map->grid_resolution().v());
} /* unit_dim_factor_calc() */

subtarget* structure3D::parent_subtarget(const ct_coord& coord) {
  size_t index = 0;
  size_t offset = 0;
  if (coord_relativity::ekVORIGIN == coord.relative_to) {
    offset = vshell_sized();
  }
  if (rmath::radians::kZERO == orientation() ||
      rmath::radians::kPI == orientation()) {
    index = (coord.offset.y() - offset) / kSUBTARGET_CELL_WIDTH;
  } else if (rmath::radians::kPI_OVER_TWO == orientation() ||
             rmath::radians::kTHREE_PI_OVER_TWO == orientation()) {
    index = (coord.offset.x() - offset) / kSUBTARGET_CELL_WIDTH;
  } else {
    ER_FATAL_SENTINEL("Bad orientation: %s",
                      rcppsw::to_string(orientation()).c_str());
  }
  ER_ASSERT(
      index <= m_subtargetsno.size(), "Bad index %zu: no such subtarget", index);
  return m_subtargetsno[index];
} /* parent_subtarget() */

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
    } /* for(j..) */
  } /* for(i..) */
} /* reset() */

void structure3D::reset_metrics(void) {
  m_placed_since_reset = 0;
  for (auto* t : subtargets()) {
    t->reset_metrics();
  } /* for(*t..) */
} /* reset_metrics() */

bool structure3D::orientation_valid(const rmath::radians& orientation) const {
  return (rmath::radians::kZERO == orientation ||
          rmath::radians::kPI_OVER_TWO == orientation ||
          rmath::radians::kPI == orientation ||
          rmath::radians::kTHREE_PI_OVER_TWO == orientation);
} /* orientation_valid() */

/*******************************************************************************
 * Progress Metrics
 ******************************************************************************/
size_t structure3D::n_total_placed(void) const { return m_placed.size(); }
size_t structure3D::n_interval_placed(void) const { return m_placed_since_reset; }

size_t structure3D::manifest_size(void) const { return m_cell_spec_map.size(); }

bool structure3D::is_complete(void) const {
  /*
   * @todo This is horribly simplistic and almost assuredly will need to be
   * updated substantially in the near future.
   */
  return n_total_placed() == manifest_size();
} /* is_complete() */

NS_END(structure, silicon);
