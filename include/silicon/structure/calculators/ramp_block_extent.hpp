/**
 * \file ramp_block_extent.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_CALCULATORS_RAMP_BLOCK_EXTENT_HPP_
#define INCLUDE_SILICON_STRUCTURE_CALCULATORS_RAMP_BLOCK_EXTENT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/math/vector3.hpp"
#include "rcppsw/er/client.hpp"

#include "silicon/silicon.hpp"
#include "silicon/structure/config/structure3D_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, calculators);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ramp_block_extent
 * \ingroup structure calculators
 *
 * \brief Give the specification for a ramp block within a target structure,
 * calculate its extent within the structure (i.e., the additional cells which
 * are not the host cell for the block, but that also are filled by the
 * volumetric extent of the block, given its size).
 */
class ramp_block_extent : rer::client<ramp_block_extent> {
 public:
  /**
   * \brief Size of the extent for ramp blocks, based what is hard-coded in
   * COSM. If that changes, then this will need to change too.
   */
  static constexpr const size_t kEXTENT_SIZE = 2;

  ramp_block_extent(void);

  /* Not move/copy constructable/assignable by default */
  ramp_block_extent(const ramp_block_extent&) = delete;
  ramp_block_extent& operator=(const ramp_block_extent&) = delete;
  ramp_block_extent(ramp_block_extent&&) = delete;
  ramp_block_extent& operator=(ramp_block_extent&&) = delete;

  std::vector<rmath::vector3z> operator()(
      const config::ramp_block_loc_spec* spec) const;
};

NS_END(calculators, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_CALCULATORS_RAMP_BLOCK_EXTENT_HPP_ */
