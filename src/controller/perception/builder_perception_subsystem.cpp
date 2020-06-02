/**
 * \file builder_perception_subsystem.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/controller/perception/builder_perception_subsystem.hpp"

#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

#include "silicon/controller/perception/perception_receptor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, controller, perception);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
builder_perception_subsystem::builder_perception_subsystem(
    const cspconfig::perception_config* const pconfig,
    const csubsystem::sensing_subsystemQ3D* const sensing)
    : base_perception_subsystem(pconfig),
      mc_arena_res(pconfig->occupancy_grid.resolution),
      mc_arena_xrange(0.0, pconfig->occupancy_grid.dims.x()),
      mc_arena_yrange(0.0, pconfig->occupancy_grid.dims.y()),
      mc_sensing(sensing),
      m_receptor(nullptr) {}

builder_perception_subsystem::~builder_perception_subsystem(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void builder_perception_subsystem::update() {
  /* Open to extension (I will probably need to put stuff here eventually...) */
} /* update() */

boost::optional<rmath::ranged> builder_perception_subsystem::ct_xrange(
    void) const {
  auto* target = nearest_ct();
  if (nullptr != target) {
    return boost::make_optional(target->xrange());
  }
  return boost::none;
} /* ct_xrange() */

boost::optional<rmath::ranged> builder_perception_subsystem::ct_yrange(
    void) const {
  auto* target = nearest_ct();
  if (nullptr != target) {
    return boost::make_optional(target->yrange());
  }
  return boost::none;
} /* ct_yrange() */

const scperception::ct_skel_info* builder_perception_subsystem::nearest_ct(
    void) const {
  return m_receptor->nearest_ct(mc_sensing->rpos3D());
} /* nearest_ct() */

void builder_perception_subsystem::receptor(
    std::unique_ptr<perception_receptor> receptor) {
  m_receptor = std::move(receptor);
} /* receptor() */

NS_END(perception, controller, silicon);
