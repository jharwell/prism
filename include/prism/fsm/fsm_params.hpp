/**
 * \file fsm_params.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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
#include "cosm/spatial/fsm/fsm_params.hpp"

#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace prism::controller::perception {
class builder_perception_subsystem;
} /* namespace prism::perception */

NS_START(prism, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
struct fsm_params  : public csfsm::fsm_params {
  const pcperception::builder_perception_subsystem* perception;
};

NS_END(fsm, prism);
