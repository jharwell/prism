/**
 * \file tv_manager.hpp
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

#ifndef INCLUDE_PRISM_SUPPORT_TV_TV_MANAGER_HPP_
#define INCLUDE_PRISM_SUPPORT_TV_TV_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/tv/tv_manager.hpp"

#include "prism/support/tv/env_dynamics.hpp"
#include "prism/support/tv/prism_pd_adaptor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace prism::controller {
class constructing_controller;
} /* namespace prism::controller */

NS_START(prism, support, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief Adapts the \ref ctv::tv_manager from COSM to PRISM.
 */
using tv_manager = ctv::tv_manager<env_dynamics, prism_pd_adaptor>;

NS_END(tv, support, prism);

#endif /* INCLUDE_PRISM_SUPPORT_TV_TV_MANAGER_HPP_ */
