/**
 * \file builder_perception_subsystem.hpp
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

#ifndef INCLUDE_SILICON_CONTROLLER_PERCEPTION_BUILDER_PERCEPTION_SUBSYSTEM_HPP_
#define INCLUDE_SILICON_CONTROLLER_PERCEPTION_BUILDER_PERCEPTION_SUBSYSTEM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/controller/perception/base_perception_subsystem.hpp"

#include "silicon/repr/builder_los.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace silicon::structure {
class structure3D;
} /* namespace silicon::structure */

NS_START(silicon, controller, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class builder_perception_subsystem
 * \ingroup controller perception
 *
 * \brief Base class for robot perception common to all builder controllers. It
 * is memory less; that is, it just stores the current LOS and allows the robot
 * to query it.
 */
class builder_perception_subsystem : public ccperception::base_perception_subsystem<repr::builder_los> {
 public:
   explicit builder_perception_subsystem(
       const ccontconfig::perception::perception_config* const pconfig)
      : base_perception_subsystem(pconfig),
        mc_arena_xrange(0.0, pconfig->occupancy_grid.dims.x()),
        mc_arena_yrange(0.0, pconfig->occupancy_grid.dims.y()) {}

  ~builder_perception_subsystem(void) override = default;

  /* not copy constructible or copy assignable by default */
  builder_perception_subsystem(const builder_perception_subsystem&) = delete;
  builder_perception_subsystem& operator=(const builder_perception_subsystem&) = delete;

  cds::block3D_vectorno blocks(void) const { return los()->blocks(); }

  /**
   * \brief Update the internal data structure/repr of the environment/arena,
   * after the LOS has been updated (for derived class controllers which
   * maintain state and need such functionality).
   */
  virtual void update(void);

  const rtypes::discretize_ratio& grid_resolution(void) const;
  const rmath::ranged& arena_xrange(void) const { return mc_arena_xrange; }
  const rmath::ranged& arena_yrange(void) const { return mc_arena_yrange; }
  rmath::ranged structure_xrange(void) const;
  rmath::ranged structure_yrange(void) const;

  const structure::structure3D* target(void) const { return mc_target; }

 private:
  /* clang-format off */
  const rmath::ranged           mc_arena_xrange;
  const rmath::ranged           mc_arena_yrange;
  const structure::structure3D* mc_target{nullptr};
  /* clang-format on */
};

NS_END(perception, controller, silicon);

#endif /* INCLUDE_SILICON_CONTROLLER_PERCEPTION_BUILDER_PERCEPTION_SUBSYSTEM_HPP_ */
