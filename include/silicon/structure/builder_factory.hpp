/**
 * \file builder_factory.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_BUILDER_FACTORY_HPP_
#define INCLUDE_SILICON_STRUCTURE_BUILDER_FACTORY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/patterns/factory/factory.hpp"
#include "silicon/silicon.hpp"
#include "silicon/structure/base_structure3D_builder.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class builder_factory
 * \ingroup fsm expstrat
 *
 * \brief Factory for creating \ref structure3D builders (duh).
 */
class builder_factory :
    public rpfactory::releasing_factory<base_structure3D_builder,
                                        std::string, /* key type */
                                        const config::structure3D_builder_config*,
                                        structure3D*,
                                        cpal::argos_sm_adaptor*> {
 public:
  static constexpr char kStatic[] = "static";
  static constexpr char kSwarm[] = "swarm";

  builder_factory(void);
};

NS_END(structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_BUILDER_FACTORY_HPP_ */
