/**
 * \file random_allocator.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/lane_alloc/random_allocator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, lane_alloc);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
random_allocator::random_allocator(rmath::rng* rng)
    : policy_allocator(rng),
      ER_CLIENT_INIT("silicon.lane_alloc.random_allocator") {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
size_t random_allocator::operator()(
    const std::vector<lane_geometry>& lanes) const {
  return rng()->uniform(0UL, lanes.size() - 1);
}/* operator()() */

NS_END(lane_alloc, silicon);
