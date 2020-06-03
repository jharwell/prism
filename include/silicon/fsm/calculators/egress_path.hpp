/**
 * \file egress_path.hpp
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

#ifndef INCLUDE_SILICON_FSM_CALCULATORS_EGRESS_PATH_HPP_
#define INCLUDE_SILICON_FSM_CALCULATORS_EGRESS_PATH_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/math/rng.hpp"

#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace silicon::repr {
class construction_lane;
} /* namespace silicon::repr */

namespace cosm::subsystem {
class sensing_subsystemQ3D;
} /* namespace cosm::subsystem */

namespace silicon::controller::perception {
class builder_perception_subsystem;
} /* namespace silicon::controller::perception */

NS_START(silicon, fsm, calculators);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class egress_path
 * \ingroup fsm calculators
 *
 * \brief Once a robot has reached the egress lane within its allocated
 * construction lane after placing a block on the structure, calculate a path
 * off of the 3D structure and back into the 2D arena.
 */
class egress_path : public rer::client<egress_path> {
 public:
  /*
   * Padding for the x/y range of the construction target, because the nest
   * extends a little beyond that range on the ingress/egress face, and we
   * (ideally) want to be out of the nest when we finish structure egress.
   */
  static constexpr const size_t kNEST_PADDING = 2;

  egress_path(const csubsystem::sensing_subsystemQ3D* sensing,
              const scperception::builder_perception_subsystem* perception,
              rmath::rng* rng);

  std::vector<rmath::vector2d> operator()(
      const srepr::construction_lane* lane) const;

  /* Not move/copy constructable/assignable by default */
  egress_path(const egress_path&) = delete;
  const egress_path& operator=(const egress_path&) =
      delete;
  egress_path(egress_path&&) = delete;
  egress_path& operator=(egress_path&&) = delete;

 private:
  /* clang-format off */
  const csubsystem::sensing_subsystemQ3D*           mc_sensing;
  const scperception::builder_perception_subsystem* mc_perception;

  rmath::rng*                                       m_rng;
  /* clang-format on */
};

NS_END(calculators, fsm, silicon);

#endif /* INCLUDE_SILICON_FSM_CALCULATORS_EGRESS_PATH_HPP_ */
