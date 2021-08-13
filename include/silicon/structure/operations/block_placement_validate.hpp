/**
 * \file block_placement_validate.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_OPERATIONS_BLOCK_PLACEMENT_VALIDATE_HPP_
#define INCLUDE_SILICON_STRUCTURE_OPERATIONS_BLOCK_PLACEMENT_VALIDATE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant/static_visitor.hpp>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector3.hpp"

#include "silicon/silicon.hpp"
#include "silicon/structure/ds/ct_coord.hpp"
#include "silicon/repr/placement_intent.hpp"

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
 * \class block_placement_validate
 * \ingroup structure operations
 *
 * \brief Validate that the placement of a block on \ref structure3D at the
 * specified location with the specified rotation is OK, or if it violates
 * graphical invariants or other such restrictions.
 */
class block_placement_validate : public rer::client<block_placement_validate> {
 public:
  block_placement_validate(const structure3D* structure,
                           const srepr::placement_intent& intent)
      : ER_CLIENT_INIT("silicon.structure.block_placement_validate"),
        mc_structure(structure),
        mc_intent(intent) {}

  /* Not copy constructible or copy assignment by default  */
  block_placement_validate(const block_placement_validate&) = delete;
  block_placement_validate& operator=(const block_placement_validate&) = delete;

  bool operator()(const crepr::cube_block3D* block) const;
  bool operator()(const crepr::ramp_block3D* block) const;

 private:
  /**
   * \brief Validation checks common to all types of blocks.
   */
  bool validate_common(void) const;

  /* clang-format off */
  const structure3D*            mc_structure;
  const srepr::placement_intent mc_intent;
  /* clang-format on */
};

NS_END(operations, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_OPERATIONS_BLOCK_PLACEMENT_VALIDATE_HPP_ */
