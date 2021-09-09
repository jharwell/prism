/**
 * \file lane_allocator.hpp
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

#ifndef INCLUDE_PRISM_LANE_ALLOC_LANE_ALLOCATOR_HPP_
#define INCLUDE_PRISM_LANE_ALLOC_LANE_ALLOCATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <map>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "prism/controller/perception/ct_skel_info.hpp"
#include "prism/lane_alloc/config/lane_alloc_config.hpp"
#include "prism/lane_alloc/lane_geometry.hpp"
#include "prism/lane_alloc/metrics/lane_alloc_metrics.hpp"
#include "prism/repr/construction_lane.hpp"
#include "prism/prism.hpp"
#include "prism/lane_alloc/history.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, lane_alloc);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class lane_allocator
 * \ingroup lane_alloc
 *
 * \brief Given the specification for a \ref structure 3D, its current progress,
 * robot location, etc., allocate a construction lane within the structure for
 * the calling robot to use.
 */
class lane_allocator : public rer::client<lane_allocator>,
                       public plametrics::lane_alloc_metrics {
 public:
  inline static const std::string kPolicyRandom = "random";
  inline static const std::string kPolicyLRU = "lru";
  inline static const std::string kPolicyClosest = "closest";
  inline static const std::string kPolicyMinInterference = "min_interference";

  lane_allocator(const config::lane_alloc_config* config, rmath::rng* rng);

  /* Not copy constructable/assignable by default */
  lane_allocator(const lane_allocator&) = delete;
  const lane_allocator& operator=(const lane_allocator&) = delete;

  /* lane allocation metrics */
  size_t alloc_count(const rtypes::type_uuid& target, size_t id) const override;
  void reset_metrics(void) override;

  std::unique_ptr<repr::construction_lane>
  operator()(const rmath::vector3d& robot_loc,
             const pcperception::ct_skel_info* target);

 private:
  /**
   * \brief Compute the locations of the entry point for each of the
   * construction lanes in the structure.
   */
  std::vector<lane_geometry>
  lane_locs_calc(const pcperception::ct_skel_info* target) const;

  /* clang-format off */
  const config::lane_alloc_config      mc_config;

  rmath::rng*                          m_rng;
  std::map<rtypes::type_uuid, history> m_history{};
  /* clang-format on */
};

NS_END(lane_alloc, prism);

#endif /* INCLUDE_PRISM_LANE_ALLOC_LANE_ALLOCATOR_HPP_ */
