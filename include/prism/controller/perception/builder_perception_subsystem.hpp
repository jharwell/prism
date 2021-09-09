/**
 * \file builder_perception_subsystem.hpp
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

#ifndef INCLUDE_PRISM_CONTROLLER_PERCEPTION_BUILDER_PERCEPTION_SUBSYSTEM_HPP_
#define INCLUDE_PRISM_CONTROLLER_PERCEPTION_BUILDER_PERCEPTION_SUBSYSTEM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>
#include <memory>

#include "cosm/subsystem/perception/rlos_perception_subsystem.hpp"

#include "prism/repr/builder_los.hpp"
#include "prism/controller/perception/ct_skel_info.hpp"
#include "prism/controller/perception/builder_prox_checker.hpp"
#include "prism/controller/perception/compass_orientation.hpp"
#include "prism/controller/perception/lane_traversal_state.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem {
class sensing_subsystemQ3D;
} /* namespace cosm::subsystem::sensing_subsystemQ3D */

namespace prism::repr {
class construction_lane;
} /* namespace prism::repr */

NS_START(prism, controller, perception);

class perception_receptor;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class builder_perception_subsystem
 * \ingroup controller perception
 *
 * \brief Base class for robot perception common to all builder controllers. It
 * is memory-less; that is, it just stores the current LOS and allows the robot
 * to query it.
 */
class builder_perception_subsystem : public csperception::rlos_perception_subsystem<repr::builder_los> {
 public:
  builder_perception_subsystem(const cspconfig::rlos_config* pconfig,
                               const csubsystem::sensing_subsystemQ3D* sensing);

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
  const rmath::ranged& arena_xrspan(void) const { return mc_arena_xrspan; }
  const rmath::ranged& arena_yrspan(void) const { return mc_arena_yrspan; }

  const builder_prox_checker* builder_prox(void) const {
    return & mc_builder_prox;
  }

  /**
   * \brief Find the nearest known construction target to the specified
   * position, or \p NULL if there are no known targets.
   */
  const ct_skel_info* nearest_ct(void) const;

  /**
   * \brief Calculate whether or not another robot with ABSOLUTE offset from the
   * current robot is in front of or behind it (i.e., an offset that accounts
   * for the robot's reference frame), accounting for robot orientation.
   */
  bool is_behind_self(const rmath::vector2d& framed_offset,
                      const prepr::construction_lane* lane) const;

  bool self_lane_aligned(const prepr::construction_lane* lane) const;

  /**
   * \brief Calculate the compass orientation (-X,+X,-Y,+Y), given azimuthal
   * angle.
   */
  compass_orientation self_compass_orientation(rmath::radians azimuth) const;

  bool is_aligned_with(rmath::radians azimuth,
                       const rmath::radians& target) const;

  struct lane_traversal_state
  lane_traversal_state(const prepr::construction_lane* lane,
                       const rmath::vector2d& rpos) const;

 private:
  /* clang-format off */
  const rtypes::discretize_ratio          mc_arena_res;
  const rmath::ranged                     mc_arena_xrspan;
  const rmath::ranged                     mc_arena_yrspan;
  const csubsystem::sensing_subsystemQ3D* mc_sensing;
  const builder_prox_checker              mc_builder_prox;

  std::unique_ptr<perception_receptor>    m_receptor;
  /* clang-format on */
};

NS_END(perception, controller, prism);

#endif /* INCLUDE_PRISM_CONTROLLER_PERCEPTION_BUILDER_PERCEPTION_SUBSYSTEM_HPP_ */
