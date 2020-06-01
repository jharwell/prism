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
#include <map>
#include <memory>

#include "rcppsw/ds/grid3D_overlay.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector3.hpp"

#include "cosm/ds/cell3D.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/repr/block_variant.hpp"
#include "cosm/ds/block3D_vector.hpp"

#include "silicon/silicon.hpp"
#include "silicon/structure/config/structure3D_config.hpp"
#include "silicon/structure/metrics/structure_state_metrics.hpp"
#include "silicon/structure/metrics/structure_progress_metrics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::arena {
class base_arena_map;
} /* namespace cosm::arena */

NS_START(silicon, structure);
class subtarget;

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
class structure3D final : public rds::grid3D_overlay<cds::cell3D>,
                          public metrics::structure_state_metrics,
                          public metrics::structure_progress_metrics,
                          public rer::client<structure3D> {
 public:
  using subtarget_vectorno = std::vector<subtarget*>;

  struct cell_spec {
    int state{};
    crepr::block_type block_type{};
    rmath::radians z_rotation{};
    size_t extent{0};
  };
  using rds::grid3D_overlay<cds::cell3D>::operator[];

  structure3D(const config::structure3D_config* config,
              const carena::base_arena_map* map,
              size_t id);
  ~structure3D(void) override;

  structure3D(const structure3D&) = default;
  const structure3D& operator=(const structure3D&) = delete;

  /* structure state metrics */
  std::vector<rmath::vector3z> occupied_cells(void) const override {
    return m_occupied_cells;
  }

  /* structure progress metrics */
  size_t n_placed_blocks(void) const override { return m_placed_since_reset; }
  size_t n_total_blocks(void) const override {
    return mc_config.cube_blocks.size() + mc_config.ramp_blocks.size();
  }
  /**
   * \brief Reset metric collection state for the structure.
   *
   * - \ref metrics::structure_progress_metrics for the overall structure
   * - \ref lane_alloc::metrics::lane_alloc_metrics for each construction lane
   *   within the structure.
   */

  void reset_metrics(void) override;

  rmath::vector3d originr(void) const { return mc_config.anchor; }
  rmath::vector3z origind(void) const {
    return rmath::dvec2zvec(originr(), mc_arena_grid_res.v());
  }
  size_t volumetric_size(void) const { return xdsize() * ydsize() * zdsize(); }
  const rmath::radians& orientation(void) const {
    return mc_config.orientation;
  }
  bool block_placement_valid(const crepr::block3D_variant& block,
                             const rmath::vector3z& loc,
                             const rmath::radians& z_rotation);

  rtypes::type_uuid id(void) const { return mc_id; }
  rmath::ranged xrange(void) const {
    return rmath::ranged(originr().x(), originr().x() + xrsize());
  }
  rmath::ranged yrange(void) const {
    return rmath::ranged(originr().y(), originr().y() + yrsize());
  }
  rmath::ranged zrange(void) const {
    return rmath::ranged(originr().z(), originr().z() + zrsize());
  }

  double block_unit_dim(void) const { return mc_block_unit_dim; }
  size_t unit_dim_factor(void) const { return mc_unit_dim_factor; }

  /**
   * \brief Return \c TRUE if a block with the same ID as the \p query currently
   * exists in the structure, and \c FALSE otherwise. Search is performed by
   * block LOCATION, rather than ID, because the structure changes the IDs of
   * the cloned blocks placed on the structure in order to differentiate them
   * from the blocks in the arena.
   */
  bool contains(const crepr::base_block3D* query) const;

  /**
   * \brief Return \c TRUE if the specified 2D location is within the bounds of
   * the structure, and \c FALSE otherwise.
   *
   * \param loc The location to check.
   * \param include_virtual Should the ring of "virtual" cells which surround
   *                        the structure be included in the search check?
   */

  bool contains(const rmath::vector2d& loc, bool include_virtual) const;
  /**
   * \brief Add a block to the structure after verifying its placement is valid
   * via \ref block_addition_valid().
   */
  void block_add(std::unique_ptr<crepr::base_block3D> block);

  /**
   * \brief Return \c TRUE if the structure has been completed and \c FALSE
   * otherwise.
   */
  bool is_complete(void) const;

  /**
   * \brief Given a location within the bounding box for the structure, retrieve
   * the final state the cell should be in once the structure is completed via
   * lookup (MUCH faster than having to compute it every queury in large
   * structures).
   */
  const cell_spec* cell_spec_retrieve(const rmath::vector3z& coord) const;

  /**
   * \brief For ramp blocks, compute the list of cells which will be in the
   * BLOCK_EXTENT state as part of the specified ramp block once the structure
   * is completed.
   */
  std::vector<rmath::vector3z> spec_to_block_extents(
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

  /**
   * \brief Return the 0-based index of the \ref subtarget to which the
   * specified cell belongs.
   */
  subtarget* cell_subtarget(const cds::cell3D& cell);

  const subtarget_vectorno& subtargets(void) const { return m_subtargetsno; }
  rtypes::type_uuid placement_id(void) {
    return rtypes::type_uuid(m_placement_id++);
  }

  void reset(void);

 private:
  using subtarget_vectoro = std::vector<std::unique_ptr<subtarget>>;
  using cell_spec_map_type = std::map<rmath::vector3z, cell_spec>;

  /**
   * \brief Size of the extent for ramp blocks, based what is hard-coded in
   * COSM. If that changes, then this will need to change too.
   */
  static constexpr const uint kRAMP_BLOCK_EXTENT_SIZE = 2;

  size_t unit_dim_factor_calc(const carena::base_arena_map* map) const;

  subtarget_vectoro subtargetso_calc(void) const;
  subtarget_vectorno subtargetsno_calc(void) const;

  /**
   * \brief Given a location within the bounding box for the structure, compute
   * the final state the cell should be in once the structure is completed.
   *
   * Should ONLY be called during initialization.
   */
  cell_spec cell_spec_calc(const rmath::vector3z& coord) const;

  cell_spec_map_type cell_spec_map_calc(void);

  /* clang-format off */
  const rtypes::type_uuid          mc_id;
  const double                     mc_block_unit_dim;
  const size_t                     mc_unit_dim_factor;
  const rtypes::discretize_ratio   mc_arena_grid_res;
  const config::structure3D_config mc_config;

  /**
   * How many blocks have been placed since last metric reset.
   */
  size_t                           m_placed_since_reset{0};
  size_t                           m_placement_id{0};
  cds::block3D_vectoro             m_placed{};

  /*
   * List of cells which have blocks in them. Better to keep a list at the cost
   * of a little extra space, then to recompute a list which will be pretty much
   * the same from timestep to timestep over and over, especially for larger
   * structures.
   */
  std::vector<rmath::vector3z>     m_occupied_cells{};
  cell_spec_map_type               m_cell_spec_map;
  subtarget_vectoro                m_subtargetso;
  subtarget_vectorno               m_subtargetsno;
  /* clang-format on */
};

NS_END(structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_STRUCTURE3D_HPP_ */
