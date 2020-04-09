/**
 * \file place_block.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_OPERATIONS_PLACE_BLOCK_HPP_
#define INCLUDE_SILICON_STRUCTURE_OPERATIONS_PLACE_BLOCK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant/static_visitor.hpp>

#include "rcppsw/math/vector3.hpp"
#include "rcppsw/er/client.hpp"

#include "silicon/silicon.hpp"
#include "silicon/structure/structure3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class place_block
 * \ingroup structure operations
 *
 * \brief Simple functor to add free blocks of any type from \ref
 * crepr::block3D_variant to \ref structure.
 */
class place_block : public rer::client<place_block>,
                    public boost::static_visitor<void> {
 public:
  place_block(const rmath::vector3u& cell,
              const rmath::radians& z_rotation,
              structure3D* structure)
      : ER_CLIENT_INIT("silicon.structure.operations.place_block"),
        mc_cell(cell),
        mc_z_rot(z_rotation),
        m_structure(structure) {}

  place_block(const place_block&) = delete;
  place_block& operator=(const place_block&) = delete;

  void operator()(crepr::cube_block3D* block) const;
  void operator()(crepr::ramp_block3D* block) const;

 private:
  /**
   * \brief Calculate offset from the absolute location of a cell within the
   * structure to where the block's absolute location in the arena needs to be
   * in order to have the 3D embodiment appear in right place in ARGoS.
   *
   * Due to the structure using lattice indexing (i.e. all locations within the
   * 1 unit area between [4.0,5,0] in X and Y have the same discrete
   * coordinates), and ARGoS stil needing absolute coordinates within the
   * arena.
   */
  rmath::vector3d embodiment_offset_calc(const crepr::base_block3D* block) const;

  /* clang-format off */
  const rmath::vector3u          mc_cell;
  const rmath::radians           mc_z_rot;

  structure3D*                   m_structure;
  /* clang-format on */
};

NS_END(operations, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_OPERATIONS_PLACE_BLOCK_HPP_ */
