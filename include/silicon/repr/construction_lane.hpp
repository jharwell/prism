/**
 * \file construction_lane.hpp
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

#ifndef INCLUDE_SILICON_REPR_CONSTRUCTION_LANE_HPP_
#define INCLUDE_SILICON_REPR_CONSTRUCTION_LANE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector3.hpp"

#include "cosm/ta/taskable_argument.hpp"

#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class construction_lane
 * \ingroup repr
 *
 * \brief Representation of a construction lane, as returned by \ref
 * lane_alloc::allocator.
 */
class construction_lane : public cta::taskable_argument {
 public:
  construction_lane(void) = default;
  construction_lane(size_t id,
                    const rmath::vector3d& center,
                    const rmath::radians& orientation,
                    const rmath::vector3d& ingress,
                    const rmath::vector3d& egress)
      : m_id(id),
        m_center(center),
        m_orientation(orientation),
        m_ingress(ingress),
        m_egress(egress) {}

  construction_lane& operator=(construction_lane&&) = default;

  /* Not copy constructable/assignable by default */
  construction_lane(const construction_lane&) = delete;
  const construction_lane& operator=(const construction_lane&) = delete;

  size_t id(void) const { return m_id; }
  const rmath::radians& orientation(void) const { return m_orientation; }
  const rmath::vector3d& ingress(void) const { return m_ingress; }
  const rmath::vector3d& egress(void) const { return m_egress; }
  const rmath::vector3d& center(void) const { return m_center; }

 private:
  /* clang-format off */
  size_t          m_id{0};
  rmath::vector3d m_center{};
  rmath::radians  m_orientation{};
  rmath::vector3d m_ingress{};
  rmath::vector3d m_egress{};
  /* clang-format on */
};

NS_END(repr, silicon);

#endif /* INCLUDE_SILICON_REPR_CONSTRUCTION_LANE_HPP_ */
