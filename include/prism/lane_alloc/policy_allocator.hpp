/**
 * \file policy_allocator.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/rng.hpp"

#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, lane_alloc);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class policy_allocator
 * \ingroup lane_alloc
 *
 * \brief The base policy which all construction lane allocation policies
 * inherit from, containing common functionality.
 */
class policy_allocator {
 public:
  explicit policy_allocator(rmath::rng* rng) : m_rng(rng) {}
  virtual ~policy_allocator(void) = default;

  /* Not move/copy constructable/assignable by default */
  policy_allocator(const policy_allocator&) = delete;
  policy_allocator& operator=(const policy_allocator&) = delete;
  policy_allocator(policy_allocator&&) = delete;
  policy_allocator& operator=(policy_allocator&&) = delete;

  rmath::rng* rng(void) const { return m_rng; }

 private:
  /* clang-format off */
  rmath::rng* m_rng;
  /* clang-format on */
};

NS_END(lane_alloc, prism);

