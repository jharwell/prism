/**
 * \file utils.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include "rcppsw/math/radians.hpp"

#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief Determine if the orientation of a gmt, block placement, etc., is
 * valid.
 */
static inline bool orientation_valid(const rmath::radians& orientation,
                                     double tol = rmath::kDOUBLE_EPSILON) {
  return orientation.is_equal(rmath::radians::kZERO, tol) ||
      orientation.is_equal(rmath::radians::kPI_OVER_TWO, tol)||
      orientation.is_equal(rmath::radians::kPI, tol) ||
      orientation.is_equal(rmath::radians::kTHREE_PI_OVER_TWO, tol);
}

NS_END(gmt, prism);

