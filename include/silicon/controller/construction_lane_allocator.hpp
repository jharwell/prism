/**
 * \file construction_lane_allocator.hpp
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

#ifndef INCLUDE_SILICON_CONTROLLER_CONSTRUCTION_LANE_ALLOCATOR_HPP_
#define INCLUDE_SILICON_CONTROLLER_CONSTRUCTION_LANE_ALLOCATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/math/rng.hpp"

#include "silicon/silicon.hpp"
#include "silicon/controller/config/lane_alloc_config.hpp"
#include "silicon/controller/metrics/lane_alloc_metrics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace silicon::structure {
class structure3D;
} /* namespace silicon::structure */

NS_START(silicon, controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class construction_lane_allocator
 * \ingroup controller
 *
 * \brief Given the specification for a \ref structure 3D, its current progress,
 * robot location, etc., allocate a construction lane within the structure for
 * the calling robot to use.
 */
class construction_lane_allocator : public rer::client<construction_lane_allocator>,
                                    public metrics::lane_alloc_metrics {
 public:
  static constexpr const char kPolicyRandom[] = "random";
  static constexpr const char kPolicyLRU[] = "lru";
  static constexpr const char kPolicyClosest[] = "closest";

  construction_lane_allocator(const config::lane_alloc_config* config,
                              const sstructure::structure3D* structure,
                              rmath::rng* rng);

  /* Not copy constructable/assignable by default */
  construction_lane_allocator(const construction_lane_allocator&) = delete;
  const construction_lane_allocator& operator=(const construction_lane_allocator&) = delete;

  /* lane allocation metrics */
  size_t alloc_count(size_t id) const override { return m_alloc_counts[id]; }

  size_t operator()(const rmath::vector3d& robot_loc);

 private:
  /**
   * \brief Compute the locations of the entry point for each of the
   * construction lanes in the structure.
   *
   * Uses a Reference to the \ref structure3D to be built. Kind of cheating, but
   * as long as robots don't use THIS reference to the structure to obtain
   * oracular information about construction progress, and only use it to simply
   * calculations regardless lane allocation, then I think it is OK.
   *
   * For example, not having this reference would make calculation of lane
   * locations much more awkward, as additional parameters would have to be
   * passed via XML or to the constructor to correctly calculate absolute
   * locations.
   */
  std::vector<rmath::vector3d> lane_locs_calc(
      const sstructure::structure3D* structure) const;

  /* clang-format off */
  const config::lane_alloc_config    mc_config;
  const std::vector<rmath::vector3d> mc_lane_locs;

  size_t                             m_prev_lane;
  rmath::rng*                        m_rng;
  std::vector<size_t>                m_alloc_counts{};
  /* clang-format on */
};

NS_END(controller, silicon);

#endif /* INCLUDE_SILICON_CONTROLLER_CONSTRUCTION_LANE_ALLOCATOR_HPP_ */
