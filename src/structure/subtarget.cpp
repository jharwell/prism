/**
 * \file subtarget.cpp
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
#include "silicon/structure/subtarget.hpp"

#include "silicon/structure/structure3D.hpp"
#include "silicon/algorithm/constants.hpp"
#include "silicon/structure/utils.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
subtarget::subtarget(const structure3D* ct, size_t id)
    : ER_CLIENT_INIT("silicon.structure.subtarget"),
      mc_entry(ssrepr::slice2D::spec_calc(slice_axis_calc(ct->orientation()),
                                          id * saconstants::kCT_SUBTARGET_WIDTH_CELLS,
                                          ct->vshell()),
               ct->spec()),
      mc_exit(ssrepr::slice2D::spec_calc(slice_axis_calc(ct->orientation()),
                                         id * saconstants::kCT_SUBTARGET_WIDTH_CELLS + 1,
                                         ct->vshell()),
              ct->spec()) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool subtarget::contains_cell(const rmath::vector3z& coord) const {
  return mc_entry.contains(coord) || mc_exit.contains(coord);
} /* contains_cell() */

rmath::vector3z
subtarget::slice_axis_calc(const rmath::radians& orientation) const {
  ER_ASSERT(sstructure::orientation_valid(orientation),
            "Bad orientation: '%s'",
            rcppsw::to_string(orientation).c_str());

  /* orientated in +X -> slice along Y */
  if (rmath::radians::kZERO == orientation ||
      rmath::radians::kPI == orientation) {
    return rmath::vector3z::Y;
  } /* orientated in +Y -> slice along X */
  else if (rmath::radians::kPI_OVER_TWO == orientation ||
           rmath::radians::kTHREE_PI_OVER_TWO == orientation) {
    return rmath::vector3z::X;
  }
  return {};
} /* slice_axis_calc() */

NS_END(structure, silicon);
