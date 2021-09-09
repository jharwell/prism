/**
 * \file lru_allocator.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/lane_alloc/lru_allocator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, lane_alloc);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
lru_allocator::lru_allocator(rmath::rng* rng)
    : policy_allocator(rng),
      ER_CLIENT_INIT("prism.lane_alloc.lru_allocator") {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
size_t lru_allocator::operator()(
    const std::vector<lane_geometry>& lanes,
    history* hist) const {
  return (hist->prev_alloc() + 1UL) % lanes.size();
}/* operator()() */

NS_END(lane_alloc, prism);
