/**
 * \file block_place.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_OPERATIONS_BLOCK_PLACE_HPP_
#define INCLUDE_SILICON_STRUCTURE_OPERATIONS_BLOCK_PLACE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant/static_visitor.hpp>
#include <memory>

#include "rcppsw/math/vector3.hpp"
#include "rcppsw/er/client.hpp"
#include "cosm/repr/cube_block3D.hpp"
#include "cosm/repr/ramp_block3D.hpp"
#include "cosm/pal/embodied_cube_block.hpp"
#include "cosm/pal/embodied_ramp_block.hpp"
#include "cosm/pal/embodied_block_variant.hpp"

#include "silicon/silicon.hpp"
#include "silicon/structure/ds/ct_coord.hpp"
#include "silicon/repr/placement_intent.hpp"

/*******************************************************************************
 * namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure);
class structure3D;

NS_START(operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_place
 * \ingroup structure operations
 *
 * \brief Add placed blocks of any type from \ref crepr::block3D_variant to a
 * \ref structure3D. The specified cell location must be relative to the virtual
 * origin of the structure.
 */
class block_place : public rer::client<block_place>,
                    boost::static_visitor<void> {
 public:
  block_place(const repr::placement_intent& intent,
              structure3D* structure)
      : ER_CLIENT_INIT("silicon.structure.operations.block_place"),
        mc_intent(intent),
        m_structure(structure) {}

  virtual ~block_place(void) = default;

  block_place(const block_place& other)
      : ER_CLIENT_INIT("silicon.structure.operations.block_place"),
        mc_intent(other.mc_intent),
        m_structure(other.m_structure) {}

  block_place& operator=(const block_place& other) = delete;

  /**
   * \brief Place a cube block onto the structure. This happens when:
   *
   * - Robots give up ownership of the block they are carrying to place it on
   *   the structure, which takes ownership.
   * - The \ref structure3D builder clones a block in the arena to give to the
   *   structure, which takes ownership.
   */
  cpal::embodied_block_variantno operator()(std::unique_ptr<cpal::embodied_cube_block> block) const;

  /**
   * \brief Place a ramp block onto the structure. This happens when:
   *
   * - Robots give up ownership of the block they are carrying to place it on
   *   the structure, which takes ownership.
   * - The \ref structure3D builder clones a block in the arena to give to the
   *   structure, which takes ownership.
   */
  cpal::embodied_block_variantno operator()(std::unique_ptr<cpal::embodied_ramp_block> block) const;

 private:
  /**
   * \brief Calculate offset from the absolute location of a cell within the
   * structure to where the block's absolute location in the arena needs to be
   * in order to have the 3D embodiment appear in right place in ARGoS.
   *
   * Due to the structure using lattice indexing (e.g., all locations within the
   * 1 unit area between [4.0,5,0] in X and Y have the same discrete
   * coordinates), and ARGoS still needs absolute coordinates within the arena.
   */
  rmath::vector3d embodiment_offset_calc(const crepr::base_block3D* block) const;

  /* clang-format off */
  const repr::placement_intent   mc_intent;

  structure3D*                   m_structure;
  /* clang-format on */
};

NS_END(operations, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_OPERATIONS_BLOCK_PLACE_HPP_ */
