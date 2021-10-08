/**
 * \file ct_vector.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <vector>

#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace prism::gmt {
class spc_gmt;
} /* namespace prism::gmt */

NS_START(prism, ds);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
using ct_vectoro = std::vector<std::unique_ptr<gmt::spc_gmt>>;

using ct_vectorno = std::vector<gmt::spc_gmt*>;

using ct_vectorro = std::vector<const gmt::spc_gmt*>;

NS_END(ds, prism);

