/**
 * \file swarm_structure3D_builder.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_SWARM_STRUCTURE3D_BUILDER_HPP_
#define INCLUDE_SILICON_STRUCTURE_SWARM_STRUCTURE3D_BUILDER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/structure/base_structure3D_builder.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class swarm_structure3D_builder
 * \ingroup structure
 *
 * \brief Action class for building structures via robot behavior.
 */
class swarm_structure3D_builder : public base_structure3D_builder {
 public:
  swarm_structure3D_builder(const config::structure3D_builder_config* config,
                            structure3D* target,
                            cpal::argos_sm_adaptor* sm)
      : base_structure3D_builder(config, target, sm) {}

  /* Not copy constructable/assignable by default */
  swarm_structure3D_builder(const swarm_structure3D_builder&) = delete;
  const swarm_structure3D_builder&
  operator=(const swarm_structure3D_builder&) = delete;

  /* parent class overrides */
  void reset(void) override {}
  void update(const rtypes::timestep&, const cds::block3D_vectorno&) override {}

 private:
  /* clang-format off */
  /* clang-format on */
};

NS_END(structure, silicon);

#endif /* INCLUDE_SILICON_SWARM_STRUCTURE_STRUCTURE3D_BUILDER_HPP_ */
