/**
 * \file base_strategy.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector3.hpp"

#include "prism/repr/fs_acq_result.hpp"

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

NS_START(prism, fsm, calculators, fs_acq);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_strategy
 * \ingroup fsm calculators fs_acq
 *
 * \brief Base class for the classes which calculate stygmergic configurations
 * in different ways.
 */
class base_strategy : public rer::client<base_strategy> {
 public:
  base_strategy(const csubsystem::sensing_subsystemQ3D* sensing,
              const pcperception::builder_perception_subsystem* perception)
      : ER_CLIENT_INIT("prism.fsm.base_strategy"),
        mc_sensing(sensing),
        mc_perception(perception) {}

  /* Not move/copy constructable/assignable by default */
  base_strategy(const base_strategy&) = delete;
  const base_strategy& operator=(const base_strategy&) = delete;
  base_strategy(base_strategy&&) = delete;
  base_strategy& operator=(base_strategy&&) = delete;

  virtual prepr::fs_acq_result operator()(
      const prepr::construction_lane* lane) const = 0;

  const auto* sensing(void) const { return mc_sensing; }
  const auto* perception(void) const { return mc_perception; }

 private:
  /* clang-format off */
  const csubsystem::sensing_subsystemQ3D*           mc_sensing;
  const pcperception::builder_perception_subsystem* mc_perception;
  /* clang-format on */
};

NS_END(fs_acq, calculators, fsm, prism);

