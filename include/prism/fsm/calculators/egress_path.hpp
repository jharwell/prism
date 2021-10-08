/**
 * \file egress_path.hpp
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
#include <vector>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/math/rng.hpp"

#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace prism::repr {
class construction_lane;
} /* namespace prism::repr */

namespace cosm::subsystem {
class sensing_subsystemQ3D;
} /* namespace cosm::subsystem */

namespace prism::controller::perception {
class builder_perception_subsystem;
} /* namespace prism::controller::perception */

NS_START(prism, fsm, calculators);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class egress_path
 * \ingroup fsm calculators
 *
 * \brief Once a robot has reached the egress lane within its allocated
 * construction lane after placing a block on the gmt, calculate a path
 * off of the 3D structure and back into the 2D arena.
 */
class egress_path : public rer::client<egress_path> {
 public:
  /*
   * Padding for the x/y range of the construction target, because the nest
   * extends a little beyond that range on the ingress/egress face, and we
   * (ideally) want to be out of the nest when we finish structure egress.
   */
  static constexpr const size_t kNEST_PADDING = 4;

  egress_path(const csubsystem::sensing_subsystemQ3D* sensing,
              const pcperception::builder_perception_subsystem* perception,
              rmath::rng* rng);

  std::vector<rmath::vector2d> operator()(
      const prepr::construction_lane* lane) const;

  /* Not move/copy constructable/assignable by default */
  egress_path(const egress_path&) = delete;
  const egress_path& operator=(const egress_path&) =
      delete;
  egress_path(egress_path&&) = delete;
  egress_path& operator=(egress_path&&) = delete;

 private:
  /* clang-format off */
  const csubsystem::sensing_subsystemQ3D*           mc_sensing;
  const pcperception::builder_perception_subsystem* mc_perception;

  rmath::rng*                                       m_rng;
  /* clang-format on */
};

NS_END(calculators, fsm, prism);

