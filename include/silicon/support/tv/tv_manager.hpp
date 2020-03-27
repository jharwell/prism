/**
 * \file tv_manager.hpp
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

#ifndef INCLUDE_SILICON_SUPPORT_TV_TV_MANAGER_HPP_
#define INCLUDE_SILICON_SUPPORT_TV_TV_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/tv/tv_manager.hpp"

#include "silicon/support/tv/env_dynamics.hpp"
#include "silicon/support/tv/silicon_pd_adaptor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace silicon::controller {
class constructing_controller;
} /* namespace silicon::controller */

NS_START(silicon, support, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief Adapts the \ref ctv::tv_manager from COSM to SILICON.
 */
using tv_manager = ctv::tv_manager<env_dynamics, silicon_pd_adaptor>;

NS_END(tv, support, silicon);

#endif /* INCLUDE_SILICON_SUPPORT_TV_TV_MANAGER_HPP_ */
