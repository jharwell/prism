/**
 * \file controller_fwd.hpp
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

#ifndef INCLUDE_SILICON_CONTROLLER_CONTROLLER_FWD_HPP_
#define INCLUDE_SILICON_CONTROLLER_CONTROLLER_FWD_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include "rcppsw/mpl/typelist.hpp"

#include "silicon/silicon.hpp"

/*******************************************************************************
 * Macros
 ******************************************************************************/
#define NON_ORACULAR_CONTROLLER_TYPES controller::fcrw_bst_controller

#define ORACULAR_CONTROLLER_TYPES

#define CONTROLLER_TYPES \
  ORACULAR_CONTROLLER_TYPES, NON_ORACULAR_CONTROLLER_TYPES

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, controller);

class fcrw_bst_controller;
class constructing_controller;
using typelist = rmpl::typelist<fcrw_bst_controller>;

NS_END(controller, silicon);

#endif /* INCLUDE_SILICON_CONTROLLER_CONTROLLER_FWD_HPP_ */
