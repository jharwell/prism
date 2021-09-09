/**
 * \file base_spct_builder.cpp
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
#include "prism/gmt/base_spct_builder.hpp"

#include <boost/ref.hpp>
#include <typeindex>

#include "cosm/arena/base_arena_map.hpp"
#include "cosm/pal/argos_sm_adaptor.hpp"
#include "cosm/pal/block_embodiment_creator.hpp"
#include "cosm/pal/embodied_block_creator.hpp"
#include "cosm/repr/cube_block3D.hpp"
#include "cosm/repr/ramp_block3D.hpp"

#include "prism/gmt/operations/block_embodiment_set.hpp"
#include "prism/gmt/operations/block_place.hpp"
#include "prism/gmt/spc_gmt.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_spct_builder::base_spct_builder(
    const config::spct_builder_config*,
    spc_gmt* target,
    cpal::argos_sm_adaptor* sm)
    : ER_CLIENT_INIT("prism.gmt.builder"), m_target(target), m_sm(sm) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool base_spct_builder::place_block(const crepr::base_block3D* block,
                                    const prepr::placement_intent& intent) {
  /* verify block addition to structure is OK */
  auto variantno = crepr::make_variant(block);
  if (!m_target->block_placement_valid(variantno, intent)) {
    ER_WARN("Block placement with intent %s failed validation: abort placement",
            rcppsw::to_string(intent).c_str());
    return false;
  }

  /*
   * Create embodied block from the foraged block. It does not yet have its
   * embodiment attached. Clones the block internally.
   *
   * Cloning the block is necessary because the structure is taking ownership
   * of the block, and you can't share ownership with the arena, which already
   * owns it.
   *
   * This also makes the # of blocks discoverable by robots in the arena as
   * construction progresses constant. See PRISM#22.
   */
  auto embodiedo =
      boost::apply_visitor(cpal::embodied_block_creator(m_sm), variantno);

  /* Add block to structure. */
  auto placement_op = operations::block_place(intent, m_target);
  cpal::embodied_block_variantno embodiedno =
      boost::apply_visitor(placement_op, std::move(embodiedo));

  /*
   * Create and set block embodiment. This MUST be after the block has been
   * placed on the structure so that the embodiment is placed at its updated
   * location.
   */
  auto creator = cpal::block_embodiment_creator(
      intent.z_rot(), m_target->placement_id(), m_sm);
  auto embodiment = boost::apply_visitor(creator, embodiedno);

  auto setter = operations::block_embodiment_set(std::move(embodiment));
  boost::apply_visitor(setter, embodiedno);
  return true;
} /* place_block() */

bool base_spct_builder::block_placement_valid(
    const crepr::block3D_variantro& block,
    const prepr::placement_intent& intent) const {
  return m_target->block_placement_valid(block, intent);
} /* block_placement_valid() */

rtypes::type_uuid base_spct_builder::target_id(void) const {
  return m_target->id();
} /* target_id() */

NS_END(gmt, prism);
