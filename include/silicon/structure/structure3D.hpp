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
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rcppsw/ds/grid3D_overlay.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/types/spatial_dist.hpp"

#include "cosm/ds/block3D_vector.hpp"
#include "cosm/ds/cell3D.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/repr/block_variant.hpp"

#include "silicon/repr/placement_intent.hpp"
#include "silicon/silicon.hpp"
#include "silicon/structure/config/structure3D_config.hpp"
#include "silicon/structure/ds/ct_coord.hpp"
#include "silicon/structure/metrics/ct_progress_metrics.hpp"
#include "silicon/structure/metrics/ct_state_metrics.hpp"

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
 *
 * Two cells of padding surround the 3D structure in a virtual "shell"; this is
 * to make the implementation of the algorithm which utilizes robot LOS correct
 * in all stygmergic configurations, so the size of the structure return by \ref
 * xdsize(), \ref ydsize() etc. \a includes these padded cells.
 */
class structure3D final : public rds::grid3D_overlay<cds::cell3D>,
                          public metrics::ct_state_metrics,
                          public metrics::ct_progress_metrics,
                          public rer::client<structure3D> {
 public:
  using subtarget_vectorno = std::vector<subtarget*>;

  struct cell_spec {
    int state{};
    crepr::block_type block_type{};
    rmath::radians z_rotation{};
    size_t extent{ 0 };
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
  bool is_complete(void) const override;
  size_t n_total_placed(void) const override;
  size_t n_interval_placed(void) const override;
  size_t manifest_size(void) const override;

  /**
   * \brief Reset metric collection state for the structure.
   *
   * - \ref metrics::structure_progress_metrics for the overall structure
   * - \ref lane_alloc::metrics::lane_alloc_metrics for each construction lane
   *   within the structure.
   */

  void reset_metrics(void) override;

  /**
   * \brief Get the size of the virtual shell which surrounds the target, in
   * units of # cells;
   */
  size_t vshell_sized(void) const { return 2; }

  /**
   * \brief Get the size of the virtual shell which surrounds the target, in
   * spatial distance;
   */
  rtypes::spatial_dist vshell_sizer(void) const {
    return rtypes::spatial_dist(block_unit_dim() * 2);
  }

  /**
   * \brief Get the real arena coordinates of the REAL origin of the structure,
   * NOT the coordinates of the origin of the virtual shell which surrounds the
   * structure.
   */
  rmath::vector3d roriginr(void) const {
    return voriginr() +
           rmath::vector3d(vshell_sizer().v(), vshell_sizer().v(), 0.0);
  }
  /**
   * \brief Get the discrete arena coordinates of the REALL origin of the
   * structure, NOT the coordinates of the origin of the virtual shell which
   * surrounds the structure.
   */
  rmath::vector3z rorigind(void) const {
    return vorigind() + rmath::vector3z(vshell_sized(), vshell_sized(), 0);
  }

  /**
   * \brief Get the real arena coordinates of the origin of the virtual shell
   * which surrounds the structure in X and Y.
   */
  rmath::vector3d voriginr(void) const {
    return grid3D_overlay<cds::cell3D>::originr();
  }

  /**
   * \brief Get the discrete arena coordinates of the origin of the virtual
   * shell which surrounds the structure in X and Y.
   */
  rmath::vector3z vorigind(void) const {
    return grid3D_overlay<cds::cell3D>::origind();
  }

  /**
   * \brief Return the orientation for the structure.
   *
   * If the orientation is 0, then internal coordinates for the cells comprising
   */

  const rmath::radians& orientation(void) const { return mc_config.orientation; }

  size_t volumetric_size(bool include_virtual = false) const {
    return xranged(include_virtual).span() * yranged(include_virtual).span() *
           zranged().span();
  }

  bool block_placement_valid(const crepr::block3D_variantro& block,
                             const srepr::placement_intent& intent);

  rtypes::type_uuid id(void) const { return mc_id; }
  rmath::ranged xranger(bool include_virtual = false) const {
    if (include_virtual) {
      return rmath::ranged(voriginr().x(), voriginr().x() + xrsize());
    } else {
      return rmath::ranged(roriginr().x(),
                           roriginr().x() + xrsize() - vshell_sizer().v() * 2);
    }
  }
  rmath::ranged yranger(bool include_virtual = false) const {
    if (include_virtual) {
      return rmath::ranged(voriginr().y(), voriginr().y() + yrsize());
    } else {
      return rmath::ranged(roriginr().y(),
                           roriginr().y() + yrsize() - vshell_sizer().v() * 2);
    }
  }
  rmath::rangez xranged(bool include_virtual = false) const {
    if (include_virtual) {
      return rmath::rangez(vorigind().x(), vorigind().x() + xdsize());
    } else {
      return rmath::rangez(rorigind().x(),
                           rorigind().x() + xdsize() - vshell_sized() * 2);
    }
  }
  rmath::rangez yranged(bool include_virtual = false) const {
    if (include_virtual) {
      return rmath::rangez(vorigind().y(), vorigind().y() + ydsize());
    } else {
      return rmath::rangez(rorigind().y(),
                           rorigind().y() + ydsize() - vshell_sized() * 2);
    }
  }
  rmath::rangez zranged(void) const {
    return rmath::rangez(origind().z(), origind().z() + zdsize());
  }
  rmath::ranged zranger(void) const {
    return rmath::ranged(originr().z(), originr().z() + zrsize());
  }

  double block_unit_dim(void) const { return mc_block_unit_dim; }
  size_t unit_dim_factor(void) const { return mc_unit_dim_factor; }

  /**
   * \brief Return \c TRUE if the a cell spec exists for the specified block,
   * and \c FALSE otherwise. Search is performed by block location (assumed to
   * be relative to structure virtual origin).
   */
  bool spec_exists(const crepr::base_block3D* query) const;

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
   * \brief Given a location within the bounding box for the structure, retrieve
   * the final state the cell should be in once the structure is completed via
   * lookup (MUCH faster than having to compute it every queury in large
   * structures).
   *
   * \param coord Coordinates of the desired cell
   */
  const cell_spec* cell_spec_retrieve(const ssds::ct_coord& coord) const;

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

  const rtypes::discretize_ratio& arena_grid_resolution(void) const {
    return mc_arena_grid_res;
  }



  /**
   * \brief Return the 0-based index of the \ref subtarget to which the
   * specified cell belongs.
   *
   * \param cell The coordinates of the cell to check.
   */
  subtarget* parent_subtarget(const ssds::ct_coord& coord);

  const subtarget_vectorno& subtargets(void) const { return m_subtargetsno; }
  rtypes::type_uuid placement_id(void) {
    return rtypes::type_uuid(m_placement_id++);
  }

  void reset(void);

  /* Add this back in when I switch representations */
  /* cds::cell3D& access(const ssds::ct_coord& c)  { */
  /*   return access(c.to_virtual().offset()); */
  /* } */

 private:
  /* force usage of the origin functions defined in this class */
  using grid3D_overlay<cds::cell3D>::originr;
  using grid3D_overlay<cds::cell3D>::origind;

  using subtarget_vectoro = std::vector<std::unique_ptr<subtarget>>;
  using cell_spec_map_type = std::map<rmath::vector3z, cell_spec>;

  size_t unit_dim_factor_calc(const carena::base_arena_map* map) const;

  subtarget_vectoro subtargetso_init(void) const;
  subtarget_vectorno subtargetsno_init(void) const;

  /**
   * \brief Given a location within the bounding box for the structure, compute
   * the final state the cell should be in once the structure is completed.
   *
   * Should ONLY be called during initialization.
   */
  cell_spec cell_spec_calc(const rmath::vector3z& coord) const;

  cell_spec_map_type cell_spec_map_init(void);

  /**
   * \brief Perform initialization sanity checks to check:
   *
   * - The structure orientation
   * - The unit dimension of blocks matches the resolution of the 3D grid
   * - The real/virtual origin coordinates (X,Y) are a multiple of the 3D grid
   *   resolution
   */
  bool initialization_checks(const config::structure3D_config* config) const;

  /**
   * \brief Return if the specified orientation is a valid orientation for the
   * structure. Used as an initialization check.
   */
  bool orientation_valid(const rmath::radians& orientation) const;

  bool post_completion_check(void) const;

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

  /**
   * Specification for all cells in the structure. Cannot be computed in the
   * constructor initializer list because it requires cell locations to be
   * populated, which only happens in the BODY of the constructor. Keys are
   * relative to the REAL origin of the structure, not the origin of the virtual
   * shell.
   */
  cell_spec_map_type               m_cell_spec_map{};

  /**
   * Subtargets within the structure. Cannot be computed in the constructor
   * initializer list because it requires cell locations to be populated, which
   * only happens in the BODY of the constructor.
   */
  subtarget_vectoro                m_subtargetso{};
  subtarget_vectorno               m_subtargetsno{};
  /* clang-format on */
};

NS_END(structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_STRUCTURE3D_HPP_ */
