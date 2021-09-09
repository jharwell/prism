/**
 * \file compass_orientation.hpp
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

#ifndef INCLUDE_PRISM_CONTROLLER_PERCEPTION_COMPASS_ORIENTATION_HPP_
#define INCLUDE_PRISM_CONTROLLER_PERCEPTION_COMPASS_ORIENTATION_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, controller, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
struct compass_orientation {
  bool pos_x;
  bool pos_y;
  bool neg_x;
  bool neg_y;
};

NS_END(perception, controller, prism);

#endif /* INCLUDE_PRISM_CONTROLLER_PERCEPTION_COMPASS_ORIENTATION_HPP_ */
