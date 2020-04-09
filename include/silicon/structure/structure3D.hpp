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
#include "cosm/repr/block_variant.hpp"

#include "silicon/silicon.hpp"
#include "silicon/structure/config/structure3D_config.hpp"
#include "silicon/structure/metrics/structure3D_metrics.hpp"
#include "silicon/structure/cell3D_target_state.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::arena {
template<typename T>
class base_arena_map;
} /* namespace cosm::arena */

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
class structure3D final : public rds::grid3D<cds::cell3D>,
                          public metrics::structure3D_metrics,
                          public rer::client<structure3D> {
 public:
  struct cell_final_spec {
    int state;
    crepr::block_type block_type;
    rmath::radians z_rotation;
  };
  using arena_map_type = carena::base_arena_map<crepr::base_block3D>;

  structure3D(const config::structure3D_config* config,
              const arena_map_type* map);

  structure3D(const structure3D&) = default;
  const structure3D& operator=(const structure3D&) = delete;

  using rds::grid3D<cds::cell3D>::operator[];

  /* structure3D metrics */
  const cds::block3D_vectorro& placed_blocks(void) const override {
    return m_placed;
  }

  rmath::vector3d originr(void) const { return mc_config.anchor; }
  rmath::vector3u origind(void) const {
    return rmath::vector3u(static_cast<uint>(mc_config.anchor.x()),
                           static_cast<uint>(mc_config.anchor.y()),
                           static_cast<uint>(mc_config.anchor.z()));
  }
  size_t volumetric_size(void) const { return xsize() * ysize() *zsize(); }
  bool contains(const crepr::base_block3D* query) const;

  bool block_placement_valid(const crepr::block3D_variant& block,
                             const rmath::vector3u& loc,
                             const rmath::radians& z_rotation);

  size_t n_placed_blocks(void) const { return m_placed.size(); }

  /**
   * \brief Add a block to the structure after verifying its placement is valid
   * via \ref block_addition_valid().
   */
  void block_add(const crepr::base_block3D* block);

  /**
   * \brief Return \c TRUE if the structure has been completed and \c FALSE
   * otherwise.
   */
  bool is_complete(void) const;

  /**
   * \brief Given a location within the bounding box for the structure, compute
   * the final state the cell should be in once the structure is completed.
   */
  cell_final_spec cell_spec(const rmath::vector3u& loc) const;

  /**
   * \brief For ramp blocks, compute the list of cells which will be in the
   * BLOCK_EXTENT state as part of the specified ramp block once the structure
   * is completed.
   */
  std::vector<rmath::vector3u> spec_to_block_extents(
      const config::ramp_block_loc_spec* spec) const;

  /**
   * \brief Verify cell state for block addition.
   */
  bool block_placement_cell_check(const cds::cell3D& cell) const;

  /**
   * \brief Given a cell from the structure, calculate its absolute position in
   * the arena. This is necessary to support blocks with a unit dimension that
   * is greater than the grid resolution of the arena.
   */
  rmath::vector3d cell_loc_abs(const cds::cell3D& cell) const;

 private:
  double unit_dim_factor_calc(const arena_map_type* map) const;

  /* clang-format off */
  const double                     mc_unit_dim_factor;
  const rtypes::discretize_ratio   mc_arena_grid_res;
  const config::structure3D_config mc_config;

  cds::block3D_vectorro            m_placed{};
  /* clang-format on */
};

NS_END(structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_STRUCTURE3D_HPP_ */
