/**
 * \file coherence_check.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/gmt/operations/coherence_check.hpp"

#include "prism/gmt/ds/connectivity_graph.hpp"
#include "prism/gmt/operations/regularity_check.hpp"
#include "prism/gmt/operations/connectivity_check.hpp"
#include "prism/gmt/operations/stability_check.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, operations);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool coherence_check::operator()(const pgds::connectivity_graph* graph,
                                 const pgrepr::vshell* vshell) const {
  ER_CHECK(regularity_check()(graph),
           "Spec is not regular");
  ER_CHECK(connectivity_check()(graph),
           "Spec violates connectivity properties");
  ER_CHECK(stability_check()(graph, vshell),
           "Spec does not represent a stable structure");

  return true;

error:
  return false;
} /* operator()() */


NS_END(operations, gmt, prism);
