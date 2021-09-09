/**
 * \file subtarget.hpp
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

#ifndef INCLUDE_PRISM_GMT_SUBTARGET_HPP_
#define INCLUDE_PRISM_GMT_SUBTARGET_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/gmt/metrics/subtarget_metrics.hpp"
#include "prism/gmt/repr/slice2D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt);
class spc_gmt;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class subtarget
 * \ingroup gmt
 *
 * \brief Representation of a subtarget/construction within the parent
 * gmt, which robots will use during task allocation.
 */
class subtarget : public rer::client<subtarget>,
                  public metrics::subtarget_metrics {
 public:
  /**
   * \param structure \ref spc_gmt handle.
   * \param id The UUID of the subtarget/construction lane among all
   *           lanes/subtargets.
   */
  subtarget(const spc_gmt* ct, size_t id);

  /* Not copy constructable/assignable by default */
  subtarget(const subtarget&) = delete;
  const subtarget& operator=(const subtarget&) = delete;

  /* progress metrics */
  bool is_complete(void) const override {
    return manifest_size() == n_total_placed();
  }
  size_t n_total_placed(void) const override { return m_total_placed; }
  size_t n_interval_placed(void) const override { return m_interval_placed; }
  size_t manifest_size(void) const override {
    return mc_entry.n_vertices() + mc_exit.n_vertices();
  }

  /**
   * \brief Update the count of blocks within the slice as the result of a block
   * placement. This function is provided so that you don't have to recalculate
   * the block count EVERY time, which is very expensive in large structures.
   */
  void placed_count_update(size_t c) {
    m_total_placed += c;
    m_interval_placed += c;
  }

  /**
   * \brief Return \c TRUE if the subtarget contains the specified cell
   * coordinates, and \c FALSE otherwise.
   */
  bool contains_cell(const rmath::vector3z& coord) const;

 private:
  /**
   * \brief Calculate the slice axis on the target gmt, given its
   * orientation.
   */
  rmath::vector3z slice_axis_calc(const rmath::radians& orientation) const;

  /* clang-format off */
  const pgrepr::slice2D mc_entry;
  const pgrepr::slice2D mc_exit;

  size_t                m_total_placed{0};
  size_t                m_interval_placed{0};
  /* clang-format on */
};

NS_END(gmt, prism);

#endif /* INCLUDE_PRISM_GMT_SUBTARGET_HPP_ */
