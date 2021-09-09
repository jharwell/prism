/**
 * \file subtarget.cpp
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
#include "prism/gmt/subtarget.hpp"

#include "prism/gmt/spc_gmt.hpp"
#include "prism/algorithm/constants.hpp"
#include "prism/gmt/utils.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
subtarget::subtarget(const spc_gmt* ct, size_t id)
    : ER_CLIENT_INIT("prism.gmt.subtarget"),
      mc_entry(pgrepr::slice2D::spec_calc(slice_axis_calc(ct->orientation()),
                                          id * paconstants::kCT_SUBTARGET_WIDTH_CELLS,
                                          ct->vshell()),
               ct->spec()),
      mc_exit(pgrepr::slice2D::spec_calc(slice_axis_calc(ct->orientation()),
                                         id * paconstants::kCT_SUBTARGET_WIDTH_CELLS + 1,
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
  ER_ASSERT(pgmt::orientation_valid(orientation),
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

NS_END(gmt, prism);
