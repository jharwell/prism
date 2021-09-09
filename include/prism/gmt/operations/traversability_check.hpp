/**
 * \file traversability_check.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_PRISM_GMT_OPERATIONS_TRAVERSABILITY_CHECK_HPP_
#define INCLUDE_PRISM_GMT_OPERATIONS_TRAVERSABILITY_CHECK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/prism.hpp"
#include "prism/gmt/repr/slice2D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace prism::gmt::ds {
class connectivity_graph;
} /* namespace prism::gmt::ds */

NS_START(prism, gmt, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class traversability_check
 * \ingroup gmt operations
 *
 * \brief Perform traversability checks as part of determining if a \ref
 * connectivity_graph corresponds to a constructible \ref spc_gmt by checking if
 * every \ref slice2D along the Z axis is traversible by a robot.
 */
class traversability_check : public rer::client<traversability_check> {
 public:
  traversability_check(void)
      : ER_CLIENT_INIT("prism.gmt.operations.traversability_check") {}

  /* Not move/copy constructable/assignable by default */
  traversability_check(const traversability_check&) = delete;
  traversability_check& operator=(const traversability_check&) = delete;
  traversability_check(traversability_check&&) = delete;
  traversability_check& operator=(traversability_check&&) = delete;

  bool operator()(const pgds::connectivity_graph* spec,
                  const pgrepr::vshell* vshell) const;

 private:
};

NS_END(operations, gmt, prism);

#endif /* INCLUDE_PRISM_GMT_OPERATIONS_TRAVERSABILITY_CHECK_HPP_ */
