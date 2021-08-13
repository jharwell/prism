/**
 * \file subtarget.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#ifndef INCLUDE_SILICON_STRUCTURE_SUBTARGET_HPP_
#define INCLUDE_SILICON_STRUCTURE_SUBTARGET_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/structure/metrics/subtarget_metrics.hpp"
#include "silicon/structure/repr/slice2D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure);
class structure3D;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class subtarget
 * \ingroup structure
 *
 * \brief Representation of a subtarget/construction within the parent
 * structure, which robots will use during task allocation.
 */
class subtarget : public rer::client<subtarget>,
                  public metrics::subtarget_metrics {
 public:
  /**
   * \param structure \ref structure3D handle.
   * \param id The UUID of the subtarget/construction lane among all
   *           lanes/subtargets.
   */
  subtarget(const structure3D* structure, size_t id);

  /* Not copy constructable/assignable by default */
  subtarget(const subtarget&) = delete;
  const subtarget& operator=(const subtarget&) = delete;

  /* progress metrics */
  bool is_complete(void) const override {
    return mc_manifest_size == n_total_placed();
  }
  size_t n_total_placed(void) const override { return m_total_placed; }
  size_t n_interval_placed(void) const override { return m_interval_placed; }
  size_t manifest_size(void) const override { return mc_manifest_size; }

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
   * \brief Calculate the slice axis on the target structure, given its
   * orientation.
   */
  rmath::vector3z slice_axis_calc(const rmath::radians& orientation) const;

  size_t manifest_size_calc(const ssrepr::slice2D& slice,
                            const structure3D* structure) const;

  /* clang-format off */
  const size_t          mc_id;
  const ssrepr::slice2D mc_entry;
  const ssrepr::slice2D mc_exit;
  const size_t          mc_manifest_size{0};

  size_t                m_total_placed{0};
  size_t                m_interval_placed{0};
  /* clang-format on */
};

NS_END(structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_SUBTARGET_HPP_ */
