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
bool structure3D_builder::build_static(const cds::block3D_vectorno& blocks,
                                          const rtypes::timestep& t) {
  ER_ASSERT(kBuildSrcLoop == mc_config.build_src,
            "Bad build source '%s' when calling %s",
            mc_config.build_src.c_str(),
            __FUNCTION__);

  /*
   * Nothing to do if it has not been long enough since the last time we placed
   * blocks.
   */
  if (!(t % mc_config.static_build_interval == 0)) {
    return true;
  }

  size_t start = 0;
  bool ret = true;
  for (; m_static_state.k < m_target->zsize(); ++m_static_state.k) {
    for (; m_static_state.i < m_target->xsize(); ++m_static_state.i) {
      for (; m_static_state.j < m_target->ysize(); ++m_static_state.j) {
        rmath::vector3u c(m_static_state.i,
                          m_static_state.j,
                          m_static_state.k);
        ER_DEBUG("Static build for cell %s", c.to_str().c_str());

        auto spec = m_target->cell_spec(c);
        if (cfsm::cell3D_state::ekST_HAS_BLOCK == spec.state) {
          if (m_static_state.interval_count >= mc_config.static_build_interval_count) {
            ER_DEBUG("Maximum build count (%zu) hit for interval",
                     mc_config.static_build_interval_count);
            return ret;
          }

          ER_DEBUG("Static build for block %zu/%zu hosted by cell %s",
                   m_static_state.interval_count,
                   mc_config.static_build_interval_count,
                   c.to_str().c_str());

          boost::optional<crepr::block3D_variant> block = build_block_find(spec.block_type,
                                                                           blocks,
                                                                           start++);
          ER_ASSERT(block,
                    "Could not find a block of type %d for location %s",
                    rcppsw::as_underlying(spec.block_type),
                    c.to_str().c_str());
          bool result = place_block(*block, c, spec.z_rotation);
          ER_ASSERT(result, "Failed to build block of type %d for location %s",
                    rcppsw::as_underlying(spec.block_type),
                    c.to_str().c_str());
          ret |= result;
          ++m_static_state.interval_count;
        }
      } /* for(j..) */
    } /* for(i..) */
  } /* for(k..) */
  m_static_state.interval_count = 0;
  return ret;
} /* build_static() */

bool structure3D_builder::place_block(const crepr::block3D_variant& block,
                                       const rmath::vector3u& loc,
                                       const rmath::radians& z_rotation) {
  /* verify block addition to structure is OK */
  if (!m_target->block_placement_valid(block, loc, z_rotation)) {
    return false;
  }
  /* Add block to structure */
  boost::apply_visitor(operations::place_block(loc, z_rotation, m_target),
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
        return boost::make_optional(v);
      } else if (crepr::block_type::ekRAMP == type &&
                 crepr::block_type::ekRAMP == blocks[eff_i]->md()->type()) {
        v = static_cast<crepr::ramp_block3D*>(blocks[eff_i]);
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
