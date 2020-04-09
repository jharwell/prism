/**
 * \file structure3D_builder.cpp
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
#include "silicon/structure/structure3D_builder.hpp"

#include <typeindex>

#include "cosm/pal/argos_sm_adaptor.hpp"
#include "cosm/repr/cube_block3D.hpp"
#include "cosm/repr/ramp_block3D.hpp"
#include "cosm/arena/base_arena_map.hpp"

#include "silicon/structure/structure3D.hpp"
#include "silicon/structure/operations/place_block.hpp"
#include "silicon/structure/operations/set_block_embodiment.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
structure3D_builder::structure3D_builder(
    const config::structure3D_builder_config* config,
    structure3D* target,
    cpal::argos_sm_adaptor* sm)
    : ER_CLIENT_INIT("silicon.structure.builder"),
      mc_config(*config),
      m_target(target),
      m_sm(sm) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
static_build_status structure3D_builder::build_static(
    const cds::block3D_vectorno& blocks,
    const rtypes::timestep& t) {
  ER_ASSERT(kBuildSrcLoop == mc_config.build_src,
            "Bad build source '%s' when calling %s",
            mc_config.build_src.c_str(),
            __FUNCTION__);

  /* nothing to do */
  if (m_target->is_complete()) {
    return static_build_status::ekFINISHED;
  }

  /*
   * Nothing to do if it has not been long enough since the last time we placed
   * blocks.
   */
  if (!(t % mc_config.static_build_interval == 0)) {
    return static_build_status::ekNO_INTERVAL;
  }

  size_t start = 0;
  bool single = true;
  while (m_static_state.n_built_interval < mc_config.static_build_interval_count) {
    if (m_static_state.n_cells >= m_target->volumetric_size()) {
      ER_ASSERT(m_target->is_complete(),
                "Structure incomplete after processing all cells!");
      return static_build_status::ekFINISHED;
    }
    single &= build_static_single(blocks, start++);
  } /* while(..) */

  ER_DEBUG("Built %zu blocks", m_static_state.n_built_interval);
  m_static_state.n_built_interval = 0;
  return (single) ? static_build_status::ekINTERVAL_LIMIT :
      static_build_status::ekINTERVAL_FAILURE;
} /* build_static() */

bool structure3D_builder::build_static_single(const cds::block3D_vectorno& blocks,
                                              size_t search_start) {
  size_t index = m_static_state.n_cells;
  size_t k = index / (m_target->xsize() * m_target->ysize());

  /*
   * We need to get the index into the "domain" for a 2D array; not doing the
   * modulo here results in incorrect Y indices once Z > 0.
   */
  index %= (m_target->xsize() * m_target->ysize());
  size_t i = index % (m_target->ysize());
  size_t j = index / (m_target->xsize());
  ER_TRACE("i=%zu,j=%zu,k=%zu", i, j, k);

  rmath::vector3u c(i, j, k);
  ER_DEBUG("Static build for structure cell@%s, abs cell@%s",
           c.to_str().c_str(),
           (m_target->origind() + c).to_str().c_str());

  bool ret = false;
  auto spec = m_target->cell_spec(c);
  if (cfsm::cell3D_state::ekST_HAS_BLOCK == spec.state) {
    ER_DEBUG("Build for block hosting cell%s (%zu/%zu for interval)",
             c.to_str().c_str(),
             m_static_state.n_built_interval,
             mc_config.static_build_interval_count);

    boost::optional<crepr::block3D_variant> block = build_block_find(spec.block_type,
                                                                     blocks,
                                                                     search_start);
    ER_ASSERT(block,
              "Could not find a block of type %d for location %s",
              rcppsw::as_underlying(spec.block_type),
              c.to_str().c_str());

    ret = place_block(*block, c, spec.z_rotation);
    ER_ASSERT(ret, "Failed to build block of type %d for location %s",
              rcppsw::as_underlying(spec.block_type),
              c.to_str().c_str());
    /* A block has disappeared from the arena floor */
    m_sm->floor()->SetChanged();
    ++m_static_state.n_built_interval;
  } else {
    ER_TRACE("Cell%s contains no block", c.to_str().c_str());
  }
  ++m_static_state.n_cells;
  return ret;
} /* build_static_single() */

bool structure3D_builder::place_block(const crepr::block3D_variant& block,
                                      const rmath::vector3u& cell,
                                      const rmath::radians& z_rotation) {
  /* verify block addition to structure is OK */
  if (!m_target->block_placement_valid(block, cell, z_rotation)) {
    ER_WARN("Block placement at %s,z_rot=%s failed validation: abort placement",
            cell.to_str().c_str(),
            z_rotation.to_str().c_str());
    return false;
  }
  /* Add block to structure */
  boost::apply_visitor(operations::place_block(cell,
                                               z_rotation,
                                               m_target),
                       block);

  /* Create block embodiment in ARGoS */
  crepr::embodied_block_variant embodiment = m_sm->make_embodied(block,
                                                                 z_rotation);

  /*
   * Associate the created block embodiment with the source block it
   * came from.
   */
  boost::apply_visitor(std::bind(operations::set_block_embodiment(),
                                 std::placeholders::_1,
                                 embodiment),
                       block);
  return true;
} /* place_block() */

boost::optional<crepr::block3D_variant> structure3D_builder::build_block_find(
    crepr::block_type type,
    const cds::block3D_vectorno& blocks,
    size_t start) const {
  for (size_t i = start; i < blocks.size() + start; ++i) {
    size_t eff_i = i % blocks.size();
    /*
     * Only blocks that are:
     *
     * - Not currently on the structure
     * - Not currently carried by a robot
     *
     * are available for selection.
     */
    if (!m_target->contains(blocks[eff_i]) &&
        !blocks[eff_i]->is_out_of_sight()) {
      crepr::block3D_variant v;
      if (crepr::block_type::ekCUBE == type &&
          crepr::block_type::ekCUBE == blocks[eff_i]->md()->type()) {
        v = static_cast<crepr::cube_block3D*>(blocks[eff_i]);
        ER_TRACE("Found cube build block%d", blocks[eff_i]->id().v());

        return boost::make_optional(v);
      } else if (crepr::block_type::ekRAMP == type &&
                 crepr::block_type::ekRAMP == blocks[eff_i]->md()->type()) {
        v = static_cast<crepr::ramp_block3D*>(blocks[eff_i]);
        ER_TRACE("Found ramp build block%d", blocks[eff_i]->id().v());
        return boost::make_optional(v);
      }
    }
  } /* for(i..) */
  return boost::optional<crepr::block3D_variant>();
} /* build_block_find() */

bool structure3D_builder::block_placement_valid(const crepr::block3D_variant& block,
                                                const rmath::vector3u& loc,
                                                const rmath::radians& z_rotation) const {
  return m_target->block_placement_valid(block, loc, z_rotation);
} /* block_placement_valid() */

NS_END(structure, silicon);
