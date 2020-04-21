/**
 * \file construct_target_vector.hpp
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

#ifndef INCLUDE_SILICON_DS_CONSTRUCT_TARGET_VECTOR_HPP_
#define INCLUDE_SILICON_DS_CONSTRUCT_TARGET_VECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <memory>

#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace silicon::structure {
class structure3D;
} /* namespace silicon::structure */

NS_START(silicon, ds);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
using construct_target_vectoro =
    std::vector<std::unique_ptr<structure::structure3D>>;

using construct_target_vectorno =
    std::vector<structure::structure3D*>;

using construct_target_vectorro =
    std::vector<const structure::structure3D*>;

NS_END(ds, silicon);

#endif /* INCLUDE_SILICON_DS_CONSTRUCT_TARGET_VECTOR_HPP_ */
