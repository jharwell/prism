/**
 * \file builder_los.cpp
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
 *****************************************************************************/
#include "silicon/repr/builder_los.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, repr);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
builder_los::builder_los(const rtypes::type_uuid& c_id,
                         const graph_view_type& c_view,
                         const rtypes::lattice_parameter& c_unit)
    : ER_CLIENT_INIT("silicon.repr.builder_los"),
      graph3D_los(c_id, c_view, c_unit) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/

NS_END(repr, silicon);
