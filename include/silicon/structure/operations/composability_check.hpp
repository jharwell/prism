/**
 * \file composability_check.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_OPERATIONS_COMPOSABILITY_CHECK_HPP_
#define INCLUDE_SILICON_STRUCTURE_OPERATIONS_COMPOSABILITY_CHECK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"

#include "silicon/silicon.hpp"
#include "silicon/structure/repr/slice2D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class composability_check
 * \ingroup structure operations
 *
 * \brief Determine if the layers of the \ref structure3D specified by the \ref
 * ssds::connectivity_graph are composable; that is, they can be stacked on top
 * of each other in a physically realizable way.
 */
class composability_check : public rer::client<composability_check> {
 public:
  composability_check(void);

  /* Not copy constructible or copy assignment by default  */
  composability_check(const composability_check&) = delete;
  composability_check& operator=(const composability_check&) = delete;

  bool operator()(const ssds::connectivity_graph* graph,
                  const ssrepr::vshell* vshell) const;

 private:
  /**
   * \brief Determine if the \p lower and \p upper layers at z, z+1, are
   * composable, that is, can be stacked/combined to produce a joint graph which
   * is:
   *
   * - Physically feasible
   */
  bool is_composable(const ssrepr::slice2D& lower,
                     const ssrepr::slice2D& upper,
                     const ssrepr::vshell* vshell) const;

  bool has_overhangs(const ssrepr::slice2D& lower,
                     const ssrepr::slice2D& upper) const;
};

NS_END(operations, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_OPERATIONS_COMPOSABILITY_CHECK_HPP_ */
