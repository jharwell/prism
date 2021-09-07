/**
 * \file history.hpp
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

#ifndef INCLUDE_SILICON_LANE_ALLOC_HISTORY_HPP_
#define INCLUDE_SILICON_LANE_ALLOC_HISTORY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/math/rng.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, lane_alloc);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class history
 * \ingroup lane_alloc
 *
 * \brief Tracks the history of what construction lanes have been allocated by a
 * single robot for a given structure over time, how much interference has been
 * experienced in a given lane, etc.
 */
class history {
 public:
  history(size_t n_lanes, rmath::rng* rng)
      : m_int_alloc_counts(n_lanes),
        m_cum_alloc_counts(n_lanes),
        m_interference_counts(n_lanes, rtypes::timestep(0)),
        m_prev_alloc(rng->uniform(0UL, n_lanes - 1)) {}

  void interference_mark(size_t id,
                         const rtypes::timestep& duration) {
    m_interference_counts[id] += duration;
  }

  void alloc_mark(size_t id) {
    m_int_alloc_counts[id]++;
    m_cum_alloc_counts[id]++;
    m_prev_alloc = id;
  }

  size_t prev_alloc(void) const { return m_prev_alloc; }

  const std::vector<rtypes::timestep>& interference_counts(void) const {
    return m_interference_counts;
  }

  size_t alloc_count(size_t id) const {
    if (id < m_int_alloc_counts.size()) {
      return m_int_alloc_counts[id];
    }
    return 0;
  }

  void reset_int_allocs(void) {
    std::fill(m_int_alloc_counts.begin(),
              m_int_alloc_counts.end(),
              0);
  }

 private:
  /* clang-formatt off */
  std::vector<size_t>           m_int_alloc_counts{};
  std::vector<size_t>           m_cum_alloc_counts{};
  std::vector<rtypes::timestep> m_interference_counts{};
  size_t                        m_prev_alloc;
  /* clang-formatt on */
};

NS_END(lane_alloc, silicon);

#endif /* INCLUDE_SILICON_LANE_ALLOC_HISTORY_HPP_ */
