/**
 * \file fs_acq_checker.hpp
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

#ifndef INCLUDE_PRISM_FSM_FS_ACQ_CHECKER_HPP_
#define INCLUDE_PRISM_FSM_FS_ACQ_CHECKER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector3.hpp"

#include "prism/repr/fs_configuration.hpp"
#include "prism/prism.hpp"
#include "prism/gmt/repr/block_spec.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace prism::repr {
class construction_lane;
class builder_los;
} /* namespace prism::repr */

namespace prism::controller::perception {
class builder_perception_subsystem;
} /* namespace prism::controller::perception */

namespace cosm::subsystem {
class sensing_subsystemQ3D;
} /* namespace cosm::subsystem */

NS_START(prism, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class fs_acq_checker
 * \ingroup fsm
 *
 * \brief As a robot proceeds along the path from the ingress point to the back
 * of the lane, this class checks to see if it has reached a stygmergic
 * configuration yet, and if so, what type.
 */
class fs_acq_checker : public rer::client<fs_acq_checker> {
 public:
  fs_acq_checker(const csubsystem::sensing_subsystemQ3D* sensing,
                 const pcperception::builder_perception_subsystem* perception);

  prepr::fs_configuration operator()(const prepr::construction_lane* lane) const;

  /* Not move/copy constructable/assignable by default */
  fs_acq_checker(const fs_acq_checker&) = delete;
  const fs_acq_checker& operator=(const fs_acq_checker&) = delete;
  fs_acq_checker(fs_acq_checker&&) = delete;
  fs_acq_checker& operator=(fs_acq_checker&&) = delete;

 private:
  struct acq_positions {
    rmath::vector3d ingress{};
    rmath::vector3d egress{};
  };

  struct acq_specs {
    const pgrepr::block_anchor_spec* ingress{ nullptr };
    const pgrepr::block_anchor_spec* egress{ nullptr };
  };

  struct acq_result {
    acq_positions positions{};
    acq_specs specs{};
    size_t lookahead{0};
  };

  /**
   * How far ahead of the robot's current cell location on the structure to look
   * for stygmergic configurations, in units of cells. This is a critical value
   * for overall builder FSM provable correctness.
   */
  static constexpr const size_t kDETECT_CELL_DIST_MAX = 3;

  acq_result acq_result_calc(const prepr::construction_lane* lane,
                             size_t lookahead) const;

  prepr::fs_configuration
  configuration_calc(const acq_result& result,
                     const prepr::construction_lane* lane,
                     const prepr::builder_los* los) const;

  acq_positions acq_positions_calc(const rmath::vector3d& rpos,
                                   const prepr::construction_lane* lane,
                                   size_t lookahead) const;

  bool acq_empty_lane(const prepr::construction_lane* lane,
                      const prepr::builder_los* los) const;

  /* clang-format off */
  const csubsystem::sensing_subsystemQ3D*           mc_sensing;
  const pcperception::builder_perception_subsystem* mc_perception;
  /* clang-format on */
};

NS_END(fsm, prism);

#endif /* INCLUDE_PRISM_FSM_FS_ACQ_CHECKER_HPP_ */
