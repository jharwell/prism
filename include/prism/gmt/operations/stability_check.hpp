/**
 * \file stability_check.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"

#include "prism/prism.hpp"
#include "prism/gmt/repr/slice2D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class stability_check
 * \ingroup gmt operations
 *
 * \brief Determine if the layers of the \ref spc_gmt specified by the \ref
 * pgds::connectivity_graph are composable; that is, they can be stacked on top
 * of each other in a physically realizable way. Framed as a graph coloring
 * problem.
 */
class stability_check : public rer::client<stability_check> {
 public:
  stability_check(void)
      : ER_CLIENT_INIT("prism.gmt.operations.stability_check") {}


  /* Not copy constructible or copy assignment by default  */
  stability_check(const stability_check&) = delete;
  stability_check& operator=(const stability_check&) = delete;

  bool operator()(const pgds::connectivity_graph* graph,
                  const pgrepr::vshell* vshell) const;

 private:
  /**
   * \brief Determine if the \p lower and \p upper layers at z, z+1, are
   * composable, that is, can be stacked/combined to produce a joint graph which
   * is:
   *
   * - Physically feasible
   */
  bool is_composable(const pgrepr::slice2D& lower,
                     const pgrepr::slice2D& upper,
                     const pgrepr::vshell* vshell) const;

  /**
   * \brief Determine if \p upper extends beyond \p lower in X or Y for any cell
   * within \p lower.
   */

  bool has_overhangs(const pgrepr::slice2D& lower,
                     const pgrepr::slice2D& upper) const;
};

NS_END(operations, gmt, prism);
