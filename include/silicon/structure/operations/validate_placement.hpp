/**
 * \file validate_placement.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_OPERATIONS_VALIDATE_PLACEMENT_HPP_
#define INCLUDE_SILICON_STRUCTURE_OPERATIONS_VALIDATE_PLACEMENT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant/static_visitor.hpp>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector3.hpp"

#include "silicon/silicon.hpp"
#include "silicon/structure/ct_coord.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::repr {
class cube_block3D;
class ramp_block3D;
} /* namespace cosm::repr */

namespace silicon::structure {
class structure3D;
} /* namespace silicon::structure */

NS_START(silicon, structure, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class validate_placement
 * \ingroup structure operations
 *
 * \brief Validate that the placement of a block on \ref structure3D at the
 * specified location with the specified rotation is OK, or if it violates
 * graphical invariants or other such restrictions.
 */
class validate_placement : public rer::client<validate_placement>,
                           public boost::static_visitor<bool> {
 public:
  validate_placement(const structure3D* structure,
                     const ct_coord& coord,
                     const rmath::radians& z_rotation)
      : ER_CLIENT_INIT("silicon.structure.validate_placement"),
        mc_structure(structure),
        mc_coord(coord),
        mc_z_rot(z_rotation) {}

  /* Not copy constructible or copy assignment by default  */
  validate_placement(const validate_placement&) = delete;
  validate_placement& operator=(const validate_placement&) = delete;

  bool operator()(const crepr::cube_block3D* block) const;
  bool operator()(const crepr::ramp_block3D* block) const;

 private:
  /**
   * \brief Validation checks common to all types of blocks.
   */
  bool validate_common(void) const;

  /* clang-format off */
  const structure3D*    mc_structure;
  const ct_coord        mc_coord;
  const rmath::radians  mc_z_rot;
  /* clang-format on */
};

NS_END(operations, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_OPERATIONS_VALIDATE_PLACEMENT_HPP_ */
