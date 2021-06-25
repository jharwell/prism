/**
 * \file random_allocator.hpp
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

#ifndef INCLUDE_SILICON_LANE_ALLOC_RANDOM_ALLOCATOR_HPP_
#define INCLUDE_SILICON_LANE_ALLOC_RANDOM_ALLOCATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "silicon/lane_alloc/policy_allocator.hpp"
#include "silicon/lane_alloc/lane_geometry.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, lane_alloc);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class random_allocator
 * \ingroup lane_alloc
 *
 * \brief Allocates construction lanes randomly (i.e., always pick a random
 * one).
 */

class random_allocator : public policy_allocator,
                         public rer::client<random_allocator> {
 public:
  explicit random_allocator(rmath::rng* rng);

  /* Not move/copy constructable/assignable by default */
  random_allocator(const random_allocator&) = delete;
  random_allocator& operator=(const random_allocator&) = delete;
  random_allocator(random_allocator&&) = delete;
  random_allocator& operator=(random_allocator&&) = delete;

  size_t operator()(const std::vector<lane_geometry>& lanes) const;

 private:
  /* clang-format off */
  /* clang-format on */
};

NS_END(lane_alloc, silicon);

#endif /* INCLUDE_SILICON_LANE_ALLOC_RANDOM_ALLOCATOR_HPP_ */
