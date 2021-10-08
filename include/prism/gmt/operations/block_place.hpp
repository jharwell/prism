/**
 * \file block_place.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant/static_visitor.hpp>
#include <memory>

#include "rcppsw/math/vector3.hpp"
#include "rcppsw/er/client.hpp"
#include "cosm/repr/cube_block3D.hpp"
#include "cosm/repr/ramp_block3D.hpp"
#include "cosm/argos/embodied_cube_block.hpp"
#include "cosm/argos/embodied_ramp_block.hpp"
#include "cosm/argos/embodied_block_variant.hpp"

#include "prism/prism.hpp"
#include "prism/gmt/repr/ct_coord.hpp"
#include "prism/repr/placement_intent.hpp"

/*******************************************************************************
 * namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt);
class spc_gmt;

NS_START(operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_place
 * \ingroup gmt operations
 *
 * \brief Add placed blocks of any type from \ref crepr::block3D_variant to a
 * \ref spc_gmt. The specified cell location must be relative to the virtual
 * origin of the structure.
 */
class block_place : public rer::client<block_place>,
                    boost::static_visitor<void> {
 public:
  block_place(const prepr::placement_intent& intent,
              spc_gmt* structure)
      : ER_CLIENT_INIT("prism.gmt.operations.block_place"),
        mc_intent(intent),
        m_structure(structure) {}

  virtual ~block_place(void) = default;

  block_place(const block_place& other)
      : ER_CLIENT_INIT("prism.gmt.operations.block_place"),
        mc_intent(other.mc_intent),
        m_structure(other.m_structure) {}

  block_place& operator=(const block_place& other) = delete;

  /**
   * \brief Place a cube block onto the structure. This happens when:
   *
   * - Robots give up ownership of the block they are carrying to place it on
   *   the gmt, which takes ownership.
   * - The \ref spc_gmt builder clones a block in the arena to give to the
   *   gmt, which takes ownership.
   */
  cargos::embodied_block_variantno operator()(std::unique_ptr<cargos::embodied_cube_block> block) const;

  /**
   * \brief Place a ramp block onto the structure. This happens when:
   *
   * - Robots give up ownership of the block they are carrying to place it on
   *   the gmt, which takes ownership.
   * - The \ref spc_gmt builder clones a block in the arena to give to the
   *   gmt, which takes ownership.
   */
  cargos::embodied_block_variantno operator()(std::unique_ptr<cargos::embodied_ramp_block> block) const;

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
  const prepr::placement_intent mc_intent;

  spc_gmt*                      m_structure;
  /* clang-format on */
};

NS_END(operations, gmt, prism);

