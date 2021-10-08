/**
 * \file perception_subsystem_factory.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include <string>

#include "cosm/subsystem/perception/config/rlos_config.hpp"

#include "rcppsw/patterns/factory/factory.hpp"
#include "prism/prism.hpp"
#include "prism/controller/perception/builder_perception_subsystem.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, controller, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class perception_subsystem_factory
 * \ingroup strategy explore
 *
 * \brief Factory for creating \ref builder_perception_subsystem derived
 * objects.
 */
class perception_subsystem_factory :
    public rpfactory::releasing_factory<builder_perception_subsystem,
                                        std::string, /* key type */
                                        const cspconfig::rlos_config*,
                                        const csubsystem::sensing_subsystemQ3D* const> {
 public:
  static inline const std::string kRLOS = "rlos";

  perception_subsystem_factory(void);
};

NS_END(perception, subsystem, prism);

