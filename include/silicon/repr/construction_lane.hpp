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
#include <boost/optional.hpp>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector3.hpp"

#include "cosm/ta/taskable_argument.hpp"

#include "silicon/lane_alloc/lane_geometry.hpp"
#include "silicon/repr/frontier_coord.hpp"
#include "silicon/silicon.hpp"
#include "silicon/lane_alloc/history.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, repr);

class builder_los;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class construction_lane
 * \ingroup repr
 *
 * \brief Representation of a construction lane, as returned by \ref
 * lane_alloc::lane_allocator.
 */
class construction_lane : public cta::taskable_argument,
                          public rer::client<construction_lane> {
 public:
  construction_lane(size_t id,
                    const rmath::radians& orientation,
                    const lane_alloc::lane_geometry& geometry,
                    lane_alloc::history* history);

  /* Not copy/move constructable/assignable by default */
  construction_lane(const construction_lane&) = delete;
  const construction_lane& operator=(const construction_lane&) = delete;

  construction_lane& operator=(construction_lane&&) = delete;
  construction_lane(const construction_lane&&) = delete;

  size_t id(void) const { return m_id; }

  bool contains(const ssds::ct_coord& coord) const;
  const rmath::radians& orientation(void) const { return m_orientation; }
  const lane_alloc::lane_geometry& geometry(void) const { return m_geometry; }
  lane_alloc::history* history(void) const { return m_history; }

 private:
  /* clang-format off */
  size_t                    m_id;
  rmath::radians            m_orientation;

  lane_alloc::lane_geometry m_geometry;
  lane_alloc::history*      m_history;
  /* clang-format on */
};

NS_END(repr, silicon);

#endif /* INCLUDE_SILICON_REPR_CONSTRUCTION_LANE_HPP_ */
