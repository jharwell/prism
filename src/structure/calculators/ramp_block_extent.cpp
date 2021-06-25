/**
 * \file ramp_block_extent.cpp
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
#include "silicon/structure/calculators/ramp_block_extent.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, calculators);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
ramp_block_extent::ramp_block_extent(void)
    : ER_CLIENT_INIT("silicon.structure.calculators.ramp_block_extent") {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::vector<rmath::vector3z>
ramp_block_extent::operator()(const config::ramp_block_loc_spec* spec) const {
  std::vector<rmath::vector3z> ret;
  /*
   * The convention is that the loc field for ramp block specs always points to
   * the anchor point/host cell of the block, and that whatever the orientation
   * of the block is (X or Y), it extends in the POSITIVE direction for that
   * orientation.
   */
  if (rmath::radians::kZERO == spec->z_rotation) {
    for (size_t m = 1; m < kEXTENT_SIZE; ++m) {
      ret.push_back({ spec->loc.x() - m, spec->loc.y(), spec->loc.z() });
    } /* for(m..) */
  } else if (rmath::radians::kPI_OVER_TWO == spec->z_rotation) {
    for (size_t m = 1; m < kEXTENT_SIZE; ++m) {
      ret.push_back({ spec->loc.x(), spec->loc.y() - m, spec->loc.z() });
    } /* for(m..) */
  } else if (rmath::radians::kPI == spec->z_rotation) {
    for (size_t m = 1; m < kEXTENT_SIZE; ++m) {
      ret.push_back({ spec->loc.x() + m, spec->loc.y(), spec->loc.z() });
    } /* for(m..) */
  } else if (rmath::radians::kTHREE_PI_OVER_TWO == spec->z_rotation) {
    for (size_t m = 1; m < kEXTENT_SIZE; ++m) {
      ret.push_back({ spec->loc.x(), spec->loc.y() + m, spec->loc.z() });
    } /* for(m..) */
  } else {
    ER_FATAL_SENTINEL("Bad rotation '%s' in spec",
                      rcppsw::to_string(spec->z_rotation).c_str());
  }
  return ret;
} /* spec_to_block_extents() */

NS_END(calculators, structure, silicon);
