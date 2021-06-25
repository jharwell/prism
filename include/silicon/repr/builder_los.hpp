/**
 * \file builder_los.hpp
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

#ifndef INCLUDE_SILICON_REPR_BUILDER_LOS_HPP_
#define INCLUDE_SILICON_REPR_BUILDER_LOS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ds/block3D_vector.hpp"
#include "cosm/repr/losQ3D.hpp"

#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class builder_los
 * \ingroup repr
 *
 * \brief A line of sight for builder robots, which computes the list 3D blocks
 * present in the LOS on request. Note that this is a 2D LOS, even though
 * building 3D structures is 3D (obviously); this is in keeping with making the
 * robots as simple as possible.
 *
 * The line of sight itself is meant to be a read-only view of part of the
 * arena, but it also exposes non-const access to the blocks within that part of
 * the arena by necessity for event processing.
 */
class builder_los final : public crepr::losQ3D, public rer::client<builder_los> {
 public:
  builder_los(const rtypes::type_uuid& c_id,
              const grid_view_type& c_view,
              const rtypes::discretize_ratio& c_resolution);

  /* not copy constructible or copy assignable by default */
  builder_los(const builder_los&) = delete;
  builder_los& operator=(const builder_los&) = delete;

  /**
   * \brief Get the list of 3D blocks currently in the LOS.
   */
  cds::block3D_vectorno blocks(void) const;
};

NS_END(repr, silicon);

#endif /* INCLUDE_SILICON_REPR_BUILDER_LOS_HPP_ */
