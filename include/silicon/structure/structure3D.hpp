/**
 * \file structure3D.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_STRUCTURE3D_HPP_
#define INCLUDE_SILICON_STRUCTURE_STRUCTURE3D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <vector>

#include "rcppsw/math/vector3.hpp"
#include "rcppsw/ds/grid3D.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/repr/base_block3D.hpp"
#include "cosm/ds/cell3D.hpp"

#include "silicon/silicon.hpp"
#include "silicon/structure/config/structure3D_config.hpp"
#include "silicon/structure/placement_orientation.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class structure3D
 * \ingroup structure
 *
 * \brief Graphical representation of the 3D structure to be built. Purely a
 * representation/data structure; does not contain any operations other than
 * basic add/remove/etc.
 */
class structure3D : public rds::grid3D<cds::cell3D>,
                    public metrics::structure3D_metrics,
                    public rer::client<structure3D> {
 public:
  explicit structure3D(const config::structure3D_config* config);

  /* Not copy constructable/assignable by default */
  structure3D(const structure3D&) = delete;
  const structure3D& operator=(const structure3D&) = delete;

  using rds::grid3D<cds::cell3D>::operator[];

  /* structure3D metrics */
  const cds::block3D_vectorro& placed_blocks(void) const override {
    return m_placed;
  }


  bool block_addition_valid(const rmath::vector3u& loc,
                            placement_orientation orientation,
                            const crepr::base_block3D* block);

  size_t n_placed_blocks(void) const { return m_placed.size(); }
  void block_add(const crepr::base_block3D* block);

 private:
  /**
   * \brief Verify cell state for block addition.
   */
  bool block_addition_cell_check(const cds::cell3D& cell) const;

  /* clang-format off */
  const rmath::vector3u mc_anchor;
  const rmath::vector3u mc_bounding_box;
  const std::string     mc_orientation;

  cds::block3D_vectorro m_placed{};
  /* clang-format on */
};

NS_END(structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_STRUCTURE3D_HPP_ */
