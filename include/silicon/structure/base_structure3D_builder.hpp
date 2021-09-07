/**
 * \file base_structure3D_builder.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_BASE_STRUCTURE3D_BUILDER_HPP_
#define INCLUDE_SILICON_STRUCTURE_BASE_STRUCTURE3D_BUILDER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>
#include <memory>

#include <argos3/plugins/simulator/entities/box_entity.h>

#include "rcppsw/er/client.hpp"

#include "cosm/ds/block3D_vector.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/repr/block_variant.hpp"

#include "silicon/repr/placement_intent.hpp"
#include "silicon/silicon.hpp"
#include "silicon/structure/repr/ct_coord.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::pal {
class argos_sm_adaptor;
} /* namespace cosm::pal */
namespace cosm::arena {
class base_arena_map;
} /* namespace cosm::arena */

NS_START(silicon, structure);

class structure3D;
namespace config {
struct structure3D_builder_config;
} /* namespace config */

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_structure3D_builder
 * \ingroup structure
 *
 * \brief Base class taking blocks placed on a \ref structure3D and doing the
 * following:
 *
 * 1. Updating the \ref structure3D with the drop
 * 2. Updating ARGoS to add the new embodied entity so that robots can interact
 * with them. Up until placement on the in-progress structure, 3D blocks are
 * rendered/treated as 2D for simplicity.
 */
class base_structure3D_builder : public rer::client<base_structure3D_builder> {
 public:
  base_structure3D_builder(const config::structure3D_builder_config* config,
                           structure3D* target,
                           cpal::argos_sm_adaptor* sm);

  base_structure3D_builder(const base_structure3D_builder&) = default;
  const base_structure3D_builder&
  operator=(const base_structure3D_builder&) = delete;

  /**
   * \brief Reset the builder--NOT including the child structure.
   */
  virtual void reset(void) = 0;

  /**
   * \brief Update the child structure on this timestep. This may involve
   * actually placing blocks on the structure (\ref static_structure3D_builder),
   * or not do anything (\ref swarm_structure3D_builder),
   */
  virtual void update(const rtypes::timestep& t,
                      const cds::block3D_vectorno& blocks) = 0;

  /**
   * \brief Place the specified block onto the structure, update \ref
   * structure3D, and add physical embodied entity in simulation representing
   * the placed block.
   *
   * If the placement of the block fails validation checks, then no action is
   * performed. Ownership of the passed block has already been relinquished, so
   * even if no action is performed the passed block is still invalid after this
   * function returns.
   *
   * \param block The block to place.
   * \param coord The \p RELATIVE location of the block within the structure
   *              (i.e. its internal coordinates). A cubical block within the
   *              structure ocupies \c ONE cell, regardless of its size in the
   *              arena.
   * \param z_rotation The orientation the block should have when placed at the
   *                   specified location.
   */
  bool place_block(const crepr::base_block3D* block,
                   const srepr::placement_intent& intent);

  bool block_placement_valid(const crepr::block3D_variantro& block,
                             const srepr::placement_intent& intent) const;

  rtypes::type_uuid target_id(void) const;

 protected:
  structure3D* target(void) const { return m_target; }
  cpal::argos_sm_adaptor* sm(void) const { return m_sm; }

 private:
  /* clang-format off */
  structure3D*                             m_target;
  cpal::argos_sm_adaptor *                 m_sm;
  /* clang-format on */
};

NS_END(structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_BASE_STRUCTURE3D_BUILDER_HPP_ */
