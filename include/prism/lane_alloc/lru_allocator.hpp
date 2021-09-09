/**
 * \file lru_allocator.hpp
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

#ifndef INCLUDE_PRISM_LANE_ALLOC_LRU_ALLOCATOR_HPP_
#define INCLUDE_PRISM_LANE_ALLOC_LRU_ALLOCATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <map>

#include "prism/lane_alloc/policy_allocator.hpp"
#include "prism/lane_alloc/lane_geometry.hpp"
#include "prism/lane_alloc/history.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, lane_alloc);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class lru_allocator
 * \ingroup lane_alloc
 *
 * \brief Allocates construction lanes according to which one was picked LEAST
 * recently.
 */

class lru_allocator : public policy_allocator,
                         public rer::client<lru_allocator> {
 public:
  explicit lru_allocator(rmath::rng* rng);

  /* Not move/copy constructable/assignable by default */
  lru_allocator(const lru_allocator&) = delete;
  lru_allocator& operator=(const lru_allocator&) = delete;
  lru_allocator(lru_allocator&&) = delete;
  lru_allocator& operator=(lru_allocator&&) = delete;

  size_t operator()(const std::vector<lane_geometry>& lanes,
                    history* hist) const;

 private:
  /* clang-format off */
  /* clang-format on */
};

NS_END(lane_alloc, prism);

#endif /* INCLUDE_PRISM_LANE_ALLOC_LRU_ALLOCATOR_HPP_ */
