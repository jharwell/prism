/**
 * \file geometry_check.hpp
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

#ifndef INCLUDE_PRISM_GMT_OPERATIONS_GEOMETRY_CHECK_HPP_
#define INCLUDE_PRISM_GMT_OPERATIONS_GEOMETRY_CHECK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"

#include "prism/gmt/repr/slice2D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace prism::gmt::ds {
class connectivity_graph;
} /* namespace prism::gmt */

namespace prism::gmt::repr {
class vshell;
} /* namespace prism::gmt::repr */

NS_START(prism, gmt, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class geometry_check
 * \ingroup gmt operations
 *
 * \brief Perform basic geometry checks as part of determining if a \ref
 * connectivity_graph corresponds to a constructible \ref spc_gmt.
 *
 * These include:
 *
 * - The orientation is valid.
 *
 * - Each layer's width is evenly divisible by the construction lane width in
 *   the orientation direction.
 */
class geometry_check : public rer::client<geometry_check> {
 public:
  geometry_check(void)
      : ER_CLIENT_INIT("prism.gmt.operations.geometry_check") {}


  /* Not copy constructable/assignable by default */
  geometry_check(const geometry_check&) = delete;
  const geometry_check& operator=(const geometry_check&) = delete;

  bool operator()(const pgds::connectivity_graph* spec,
                  const pgrepr::vshell* vshell,
                  const rmath::radians& orientation) const;

 private:
  /**
   * \brief Perform geometry checks on a single layer within the \ref
   * connectivity_graph.
   */
  bool layer_check(const pgrepr::slice2D& layer,
                   const rmath::radians& orientation) const;
};

NS_END(operations, gmt, prism);

#endif /* INCLUDE_PRISM_GMT_OPERATIONS_GEOMETRY_CHECK_HPP_ */
