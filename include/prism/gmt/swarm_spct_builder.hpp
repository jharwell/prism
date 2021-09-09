/**
 * \file swarm_spct_builder.hpp
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

#ifndef INCLUDE_PRISM_GMT_SWARM_SPCT3D_BUILDER_HPP_
#define INCLUDE_PRISM_GMT_SWARM_SPCT3D_BUILDER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/gmt/base_spct_builder.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class swarm_spct_builder
 * \ingroup gmt
 *
 * \brief Action class for building structures via robot behavior.
 */
class swarm_spct_builder : public base_spct_builder {
 public:
  swarm_spct_builder(const config::spct_builder_config* config,
                     spc_gmt* target,
                     cpal::argos_sm_adaptor* sm)
      : base_spct_builder(config, target, sm) {}

  /* Not copy constructable/assignable by default */
  swarm_spct_builder(const swarm_spct_builder&) = delete;
  const swarm_spct_builder&
  operator=(const swarm_spct_builder&) = delete;

  /* parent class overrides */
  void reset(void) override {}
  void update(const rtypes::timestep&, const cds::block3D_vectorno&) override {}

 private:
  /* clang-format off */
  /* clang-format on */
};

NS_END(gmt, prism);

#endif /* INCLUDE_PRISM_SWARM_SPCT_SPCT3D_BUILDER_HPP_ */
