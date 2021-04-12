/**
 * \file allocator.hpp
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

#ifndef INCLUDE_SILICON_LANE_ALLOC_ALLOCATOR_HPP_
#define INCLUDE_SILICON_LANE_ALLOC_ALLOCATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "silicon/controller/perception/ct_skel_info.hpp"
#include "silicon/lane_alloc/config/lane_alloc_config.hpp"
#include "silicon/lane_alloc/lane_geometry.hpp"
#include "silicon/lane_alloc/metrics/lane_alloc_metrics.hpp"
#include "silicon/repr/construction_lane.hpp"
#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, lane_alloc);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class allocator
 * \ingroup lane_alloc
 *
 * \brief Given the specification for a \ref structure 3D, its current progress,
 * robot location, etc., allocate a construction lane within the structure for
 * the calling robot to use.
 */
class allocator : public rer::client<allocator>,
                  public slametrics::lane_alloc_metrics {
 public:
  static constexpr const char kPolicyRandom[] = "random";
  static constexpr const char kPolicyLRU[] = "lru";
  static constexpr const char kPolicyClosest[] = "closest";

  allocator(const config::lane_alloc_config* config, rmath::rng* rng);

  /* Not copy constructable/assignable by default */
  allocator(const allocator&) = delete;
  const allocator& operator=(const allocator&) = delete;

  /* lane allocation metrics */
  size_t alloc_count(const rtypes::type_uuid& target, size_t id) const override;
  void reset_metrics(void) override;

  std::unique_ptr<repr::construction_lane>
  operator()(const rmath::vector3d& robot_loc,
             const scperception::ct_skel_info* target);

 private:
  struct allocation_history {
    allocation_history(size_t n_lanes, size_t prev)
        : alloc_counts(n_lanes), prev_lane(prev) {}

    std::vector<size_t> alloc_counts{};
    size_t prev_lane{ 0 };
  };

  /**
   * \brief Compute the locations of the entry point for each of the
   * construction lanes in the structure.
   */
  std::vector<lane_geometry>
  lane_locs_calc(const scperception::ct_skel_info* target) const;

  /* clang-format off */
  const config::lane_alloc_config                 mc_config;

  rmath::rng*                                     m_rng;
  std::map<rtypes::type_uuid, allocation_history> m_history{};
  /* clang-format on */
};

NS_END(lane_alloc, silicon);

#endif /* INCLUDE_SILICON_LANE_ALLOC_~ALLOCATOR_HPP_ */
