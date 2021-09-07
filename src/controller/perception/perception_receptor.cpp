/**
 * \file perception_receptor.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/controller/perception/perception_receptor.hpp"

#include <algorithm>

#include "silicon/structure/structure3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, controller, perception);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
const scperception::ct_skel_info*
perception_receptor::nearest_ct(const rmath::vector3d& pos) const {
  auto pred = [&](const auto& target1, const auto& target2) {
    return (target1.roriginr() - pos).length() <
           (target2.roriginr() - pos).length();
  };
  auto it = std::min_element(m_targets.begin(), m_targets.end(), pred);
  if (m_targets.end() != it) {
    return &(*it);
  }
  return nullptr;
} /* nearest_ct() */

NS_END(perception, controller, silicon);
