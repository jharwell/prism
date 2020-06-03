/**
 * \file base_structure3D_builder.cpp
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
#include "silicon/structure/base_structure3D_builder.hpp"

#include <boost/ref.hpp>
#include <typeindex>

#include "cosm/arena/base_arena_map.hpp"
#include "cosm/pal/argos_sm_adaptor.hpp"
#include "cosm/repr/cube_block3D.hpp"
#include "cosm/repr/ramp_block3D.hpp"

#include "silicon/structure/operations/block_place.hpp"
#include "silicon/structure/operations/set_block_embodiment.hpp"
#include "silicon/structure/structure3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_structure3D_builder::base_structure3D_builder(
    const config::structure3D_builder_config*,
    structure3D* target,
    cpal::argos_sm_adaptor* sm)
    : ER_CLIENT_INIT("silicon.structure.builder"),
      m_target(target),
      m_sm(sm) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool base_structure3D_builder::place_block(std::unique_ptr<crepr::base_block3D> block,
                                      const ct_coord& coord,
                                      const rmath::radians& z_rotation) {
  /*
   * We give the placed block a unique ID that probably won't be the same as
   * the one it had in the arena; this is necessary since blocks are cloned when
   * pulling from the arena during static build and the ID of the cloned block
   * is used to create the ARGoS embodiment ID, which MUST be unique among all
   * entities.
   */
  block->id(m_target->placement_id());

  /*
   * This variant is non-owning, because ownership is not needed until we
   * actually call the place_block operation.
   */
  auto variantno = create_variant(block.get());

  /* verify block addition to structure is OK */
  if (!m_target->block_placement_valid(variantno, coord, z_rotation)) {
    ER_WARN("Block placement in cell@%s,z_rot=%s failed validation: abort placement",
            rcppsw::to_string(coord.offset).c_str(),
            rcppsw::to_string(z_rotation).c_str());
    return false;
  }
  /*
   * Add block to structure.
   *
   * boost::variant does not play nice with move-only types without lots of
   * contortions, as far as I can tell, so we need to bend the usual coding
   * conventions and use a raw pointer which indicates OWNING access within the
   * scope of the called operation.
   */
  auto varianto = create_variant(block.release());
  auto placement_op = operations::block_place(coord,
                                              z_rotation,
                                              m_target);
  boost::apply_visitor(placement_op, varianto);

  /*
   * Create block embodiment in ARGoS and associate the created block embodiment
   * with the source block it came from. This MUST be after the block has been
   * placed on the structure so that the embodiment is placed at its updated
   * location.
   */
  crepr::embodied_block_variant embodiment =
      m_sm->make_embodied(variantno, z_rotation, m_target->id());
  boost::apply_visitor(std::bind(operations::set_block_embodiment(),
                                 std::placeholders::_1,
                                 embodiment),
                       variantno);

  return true;
} /* place_block() */

bool base_structure3D_builder::block_placement_valid(
    const crepr::block3D_variant& block,
    const ct_coord& coord,
    const rmath::radians& z_rotation) const {
  return m_target->block_placement_valid(block, coord, z_rotation);
} /* block_placement_valid() */

rtypes::type_uuid base_structure3D_builder::target_id(void) const {
  return m_target->id();
} /* target_id() */

crepr::block3D_variant base_structure3D_builder::create_variant(
    crepr::base_block3D* block) const {
  if (crepr::block_type::ekCUBE == block->md()->type()) {
    return {static_cast<crepr::cube_block3D*>(block)};
  } else if (crepr::block_type::ekRAMP == block->md()->type()) {
    return {static_cast<crepr::ramp_block3D*>(block)};
  } else {
    ER_FATAL_SENTINEL("Bad 3D block type %d",
                      rcppsw::as_underlying(block->md()->type()));
    return {};
  }
} /* create_variant() */

NS_END(structure, silicon);
