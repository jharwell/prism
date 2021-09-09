/**
 * \file perception_receptor.hpp
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

#ifndef INCLUDE_PRISM_CONTROLLER_PERCEPTION_PERCEPTION_RECEPTOR_HPP_
#define INCLUDE_PRISM_CONTROLLER_PERCEPTION_PERCEPTION_RECEPTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/math/vector3.hpp"

#include "prism/prism.hpp"
#include "prism/controller/perception/ct_skel_info.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, controller, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class perception_receptor
 * \ingroup controller perception
 *
 * \brief Adaptor to bridge the gap between robot controllers and the loop
 * functions, so that information about construction targets can be
 * injected/gathered in a modular way.
 *
 * The initial implementation requires that the receptor (and therefore robots)
 * know the bounding box and anchor of each construction target at the start of
 * simulation.
 */

class perception_receptor {
 public:
  using ct_info_vector = std::vector<ct_skel_info>;
  explicit perception_receptor(const ct_info_vector& targets)
      : m_targets(targets) {}

  /* Not copy constructable/assignable by default */
  perception_receptor(const perception_receptor&) = delete;
  const perception_receptor& operator=(const perception_receptor&) = delete;

  /**
   * \brief Find the nearest \ref spc_gmt target to the given location, or
   * \p NULL if there are no known targets.
   */
  const ct_skel_info* nearest_ct(const rmath::vector3d& pos) const;

 private:
  /* clang-format off */
  ct_info_vector m_targets;
  /* clang-format on */
};

NS_END(perception, controller, prism);

#endif /* INCLUDE_PRISM_CONTROLLER_PERCEPTION_PERCEPTION_RECEPTOR_HPP_ */
