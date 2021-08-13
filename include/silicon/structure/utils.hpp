/**
 * \file utils.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_SILICON_STRUCTURE__UTILS_HPP_
#define INCLUDE_SILICON_STRUCTURE__UTILS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/radians.hpp"

#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief Determine if the orientation of a structure, block placement, etc., is
 * valid.
 */
static inline bool orientation_valid(const rmath::radians& orientation) {
  return rmath::radians::kZERO == orientation ||
      rmath::radians::kPI_OVER_TWO == orientation ||
      rmath::radians::kPI == orientation ||
      rmath::radians::kTHREE_PI_OVER_TWO == orientation;
}

NS_END(structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE__UTILS_HPP_ */
