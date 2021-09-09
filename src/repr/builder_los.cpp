/**
 * \file builder_los.cpp
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
 *****************************************************************************/
#include "prism/repr/builder_los.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, repr);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
builder_los::builder_los(const rtypes::type_uuid& c_id,
                          graph_view_type&& the_view,
                         const rtypes::spatial_dist& c_unit)
    : ER_CLIENT_INIT("prism.repr.builder_los"),
      graph3D_los(c_id, std::move(the_view), c_unit) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/

NS_END(repr, prism);
