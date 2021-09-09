/**
 * \file block_placement_validate.hpp
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

#ifndef INCLUDE_PRISM_GMT_OPERATIONS_BLOCK_PLACEMENT_VALIDATE_HPP_
#define INCLUDE_PRISM_GMT_OPERATIONS_BLOCK_PLACEMENT_VALIDATE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant/static_visitor.hpp>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector3.hpp"

#include "prism/prism.hpp"
#include "prism/gmt/repr/ct_coord.hpp"
#include "prism/repr/placement_intent.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::repr {
class cube_block3D;
class ramp_block3D;
} /* namespace cosm::repr */

namespace prism::gmt {
class spc_gmt;
} /* namespace prism::gmt */

NS_START(prism, gmt, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_placement_validate
 * \ingroup gmt operations
 *
 * \brief Validate that the placement of a block on \ref spc_gmt at the
 * specified location with the specified rotation is OK, or if it violates
 * graphical invariants or other such restrictions.
 */
class block_placement_validate : public rer::client<block_placement_validate> {
 public:
  block_placement_validate(const spc_gmt* target,
                           const prepr::placement_intent& intent)
      : ER_CLIENT_INIT("prism.gmt.block_placement_validate"),
        mc_target(target),
        mc_intent(intent) {}

  /* Not copy constructible or copy assignment by default  */
  block_placement_validate(const block_placement_validate&) = delete;
  block_placement_validate& operator=(const block_placement_validate&) = delete;

  bool operator()(const crepr::cube_block3D* block) const;
  bool operator()(const crepr::ramp_block3D* block) const;

 private:
  /**
   * \brief The tolerance for Z rotational differences between a \ref
   * prepr::placement_intent and what the spec says. Mainly needed by static
   * builders which read the Z rotation from a file, which is subject to the
   * number of digits of precision used.
   */
  static constexpr double const kZ_ROT_TOL = 0.0001;

  /**
   * \brief Validation checks common to all types of blocks.
   */
  bool validate_common(void) const;

  /* clang-format off */
  const spc_gmt*               mc_target;
  const prepr::placement_intent mc_intent;
  /* clang-format on */
};

NS_END(operations, gmt, prism);

#endif /* INCLUDE_PRISM_GMT_OPERATIONS_BLOCK_PLACEMENT_VALIDATE_HPP_ */
