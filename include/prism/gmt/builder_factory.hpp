/**
 * \file builder_factory.hpp
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
#include <string>

#include "rcppsw/patterns/factory/factory.hpp"

#include "prism/prism.hpp"
#include "prism/gmt/base_spct_builder.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class builder_factory
 * \ingroup fsm expstrat
 *
 * \brief Factory for creating \ref spc_gmt builders (duh).
 */
class builder_factory
    : public rpfactory::releasing_factory<base_spct_builder,
                                          std::string, /* key type */
                                          const config::spct_builder_config*,
                                          spc_gmt*,
                                          cpargos::swarm_manager_adaptor*> {
 public:
  inline static const std::string kStatic = "static";
  inline static const std::string kSwarm = "swarm";

  builder_factory(void);
};

NS_END(gmt, prism);
