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
#include <boost/optional.hpp>
#include <memory>

#include "cosm/subsystem/perception/base_perception_subsystem.hpp"

#include "silicon/repr/builder_los.hpp"
#include "silicon/controller/perception/ct_skel_info.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem {
class sensing_subsystemQ3D;
} /* namespace cosm::subsystem::sensing_subsystemQ3D */

NS_START(silicon, controller, perception);

class perception_receptor;

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
class builder_perception_subsystem : public csperception::base_perception_subsystem<repr::builder_los> {
 public:
  builder_perception_subsystem(const cspconfig::perception_config* const pconfig,
                               const csubsystem::sensing_subsystemQ3D* const sensing);

  ~builder_perception_subsystem(void) override;

  /* not copy constructible or copy assignable by default */
  builder_perception_subsystem(const builder_perception_subsystem&) = delete;
  builder_perception_subsystem& operator=(const builder_perception_subsystem&) = delete;

  void receptor(std::unique_ptr<perception_receptor> receptor);

  /**
   * \brief Update the internal data structure/repr of the environment/arena,
   * after the LOS has been updated (for derived class controllers which
   * maintain state and need such functionality).
   */
  virtual void update(void);

  const rtypes::discretize_ratio& arena_resolution(void) const { return mc_arena_res; }
  const rmath::ranged& arena_xrange(void) const { return mc_arena_xrange; }
  const rmath::ranged& arena_yrange(void) const { return mc_arena_yrange; }

  /**
   * \brief Return the X range of the nearest known construction target, or
   * empty if there are no known targets.
   */
  boost::optional<rmath::ranged> ct_xrange(void) const;

  /**
   * \brief Return the Y range of the nearest known construction target, or
   * empty if there are no known targets.
   */
  boost::optional<rmath::ranged> ct_yrange(void) const;

  /**
   * \brief Find the nearest known construction target to the specified
   * position, or \p NULL if there are no known targets.
   */
  const ct_skel_info* nearest_ct(void) const;

 private:
  /* clang-format off */
  const rtypes::discretize_ratio          mc_arena_res;
  const rmath::ranged                     mc_arena_xrange;
  const rmath::ranged                     mc_arena_yrange;
  const csubsystem::sensing_subsystemQ3D* mc_sensing;

  std::unique_ptr<perception_receptor>    m_receptor;
  /* clang-format on */
};

NS_END(perception, controller, silicon);

#endif /* INCLUDE_SILICON_CONTROLLER_PERCEPTION_BUILDER_PERCEPTION_SUBSYSTEM_HPP_ */
