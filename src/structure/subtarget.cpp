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

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
subtarget::subtarget(const structure3D* structure, size_t id)
    : ER_CLIENT_INIT("silicon.structure.subtarget"),
      mc_id(id),
      mc_entry(slice2D::coords_calc(slice_axis_calc(structure->orientation()),
                                    structure,
                                    id * structure3D::kSUBTARGET_CELL_WIDTH),
               structure),
      mc_exit(slice2D::coords_calc(slice_axis_calc(structure->orientation()),
                                   structure,
                                   id * structure3D::kSUBTARGET_CELL_WIDTH + 1),
              structure),
      mc_manifest_size(manifest_size_calc(mc_entry, structure) +
                       manifest_size_calc(mc_exit, structure)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool subtarget::contains_cell(const rmath::vector3z& coord) const {
  return mc_entry.contains(coord) || mc_exit.contains(coord);
} /* contains_cell() */

rmath::vector3z
subtarget::slice_axis_calc(const rmath::radians& orientation) const {
  /* orientated in +X -> slice along Y */
  if (rmath::radians::kZERO == orientation ||
      rmath::radians::kPI == orientation) {
    return rmath::vector3z::Y;
  } /* orientated in +Y -> slice along X */
  else if (rmath::radians::kPI_OVER_TWO == orientation ||
           rmath::radians::kTHREE_PI_OVER_TWO == orientation) {
    return rmath::vector3z::X;
  }
  ER_FATAL_SENTINEL("Bad orientation for slice axis calculation: '%s",
                    rcppsw::to_string(orientation).c_str());
  return {};
} /* slice_axis_calc() */

size_t subtarget::manifest_size_calc(const slice2D& slice,
                                     const structure3D* structure) const {
  size_t count = 0;
  for (size_t i = 0; i < slice.d1(); ++i) {
    for (size_t j = 0; j < slice.d2(); ++j) {
      auto coord = ssds::ct_coord{ slice.access(i, j).loc(),
                                   ssds::ct_coord::relativity::ekVORIGIN,
                                   structure };
      const auto* spec = structure->cell_spec_retrieve(coord);

      count += cfsm::cell3D_state::ekST_HAS_BLOCK == spec->state;
    } /* for(j..) */
  } /* for(i..) */
  return count;
} /* manifest_size_calc() */

NS_END(structure, silicon);
