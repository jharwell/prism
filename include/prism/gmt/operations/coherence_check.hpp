/**
 * \file coherence_check.hpp
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

#include "cosm/repr/block_variant.hpp"

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
 * \class coherence_check
 * \ingroup gmt operations
 *
 * \brief Perform checks to determine if a \ref connectivity_graph is coherent.
 */
class coherence_check : public rer::client<coherence_check> {
 public:
  coherence_check(void)
      : ER_CLIENT_INIT("prism.gmt.operations.coherence_check") {}


  /* Not copy constructable/assignable by default */
  coherence_check(const coherence_check&) = delete;
  const coherence_check& operator=(const coherence_check&) = delete;

  bool operator()(const pgds::connectivity_graph* graph,
                  const pgrepr::vshell* vshell) const;

 private:
};

NS_END(operations, gmt, prism);
