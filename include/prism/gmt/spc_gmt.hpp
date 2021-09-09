/**
 * \file spc_gmt.hpp
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

#ifndef INCLUDE_PRISM_GMT_SPC_GMT_HPP_
#define INCLUDE_PRISM_GMT_SPC_GMT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/types/spatial_dist.hpp"
#include "rcppsw/ds/graph/grid_index_map.hpp"

#include "cosm/repr/base_block3D.hpp"
#include "cosm/repr/block_variant.hpp"
#include "cosm/ds/block3D_vector.hpp"

#include "prism/repr/placement_intent.hpp"
#include "prism/prism.hpp"
#include "prism/gmt/config/spct_config.hpp"
#include "prism/gmt/repr/ct_coord.hpp"
#include "prism/gmt/metrics/ct_progress_metrics.hpp"
#include "prism/gmt/metrics/ct_state_metrics.hpp"
#include "prism/gmt/repr/block_spec.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::arena {
class base_arena_map;
} /* namespace cosm::arena */

NS_START(prism, gmt);
class subtarget;
namespace ds {
class connectivity_graph;
class block_anchor_index;
} /* namespace ds */

namespace repr {
class vshell;
} /* namespace repr */

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class spc_gmt
 * \ingroup gmt
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
class spc_gmt final : public rer::client<spc_gmt>,
                      public metrics::ct_state_metrics,
                      public metrics::ct_progress_metrics,
                      public crepr::entity3D {
 public:
  using subtarget_vectorno = std::vector<subtarget*>;

  spc_gmt(const config::spct_config* config,
          const carena::base_arena_map* map,
          const rtypes::type_uuid& id);
  ~spc_gmt(void) override;

  spc_gmt(const spc_gmt&) = delete;
  const spc_gmt& operator=(const spc_gmt&) = delete;

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
   * - \ref metrics::gmt_progress_metrics for the overall structure
   * - \ref lane_alloc::metrics::lane_alloc_metrics for each construction lane
   *   within the structure.
   */

  void reset_metrics(void) override;

  const pgrepr::vshell* vshell(void) const { return m_vshell.get(); }
  const pgds::connectivity_graph* spec(void) const { return m_spec_graph.get(); }
  const pgds::block_anchor_index* anchor_index(void) const {
    return m_anchor_index.get();
  }

  /**
   * \brief Return the orientation for the structure.
   */
  const rmath::radians& orientation(void) const { return mc_config.orientation; }

  bool block_placement_valid(const crepr::block3D_variantro& block,
                             const prepr::placement_intent& intent);

  rtypes::spatial_dist block_unit_dim(void) const { return mc_block_unit_dim; }
  size_t unit_dim_factor(void) const { return mc_unit_dim_factor; }

  /**
   * \brief Return \c TRUE if the a cell spec exists for the specified block,
   * and \c FALSE otherwise. Search is performed by block location (assumed to
   * be relative to structure virtual origin).
   */
  bool spec_exists(const crepr::base_block3D* query) const;

  /**
   * \brief Return \c TRUE if the specified 2D location is within the bounds of
   * the gmt, and \c FALSE otherwise.
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
   * \brief Given a block anchor location within the bounding box for the
   * gmt, retrieve the spec for the location (MUCH faster than having to
   * compute it every query in large structures).
   *
   * \param coord Coordinates of the desired block anchor point.
   */
  const pgrepr::block_anchor_spec* spec_retrieve(const pgrepr::ct_coord& coord) const;

  /**
   * \brief Given a \ref ds::block_anchor_spec from the gmt, calculate its
   * absolute position in the arena. This is necessary to support blocks with a
   * unit dimension that is greater than the grid resolution of the arena.
   */
  rmath::vector3d anchor_loc_abs(const pgrepr::ct_coord& anchor) const;

  const rtypes::discretize_ratio& arena_grid_resolution(void) const {
    return mc_arena_grid_res;
  }

  /**
   * \brief Return the 0-based index of the \ref subtarget to which the
   * specified cell belongs.
   *
   * \param cell The coordinates of the cell to check.
   */
  subtarget* parent_subtarget(const pgrepr::ct_coord& coord);

  const subtarget_vectorno& subtargets(void) const { return m_subtargetsno; }
  rtypes::type_uuid placement_id(void) {
    return rtypes::type_uuid(m_placement_id++);
  }

  void reset(void);

 private:
  using subtarget_vectoro = std::vector<std::unique_ptr<subtarget>>;

  static rtypes::spatial_dist block_unit_dim_calc(const cds::block3D_vectorno& blocks);
  size_t unit_dim_factor_calc(const carena::base_arena_map* map) const;

  subtarget_vectoro subtargetso_init(void) const;
  subtarget_vectorno subtargetsno_init(void) const;

  void ds_init(void);

  bool post_completion_check(void) const;

  /* clang-format off */
  const rtypes::spatial_dist                mc_block_unit_dim;
  const size_t                              mc_unit_dim_factor;
  const rtypes::discretize_ratio            mc_arena_grid_res;
  const config::spct_config                 mc_config;

  /**
   * How many blocks have been placed since last metric reset.
   */
  size_t                                    m_placed_since_reset{0};
  size_t                                    m_placed{0};
  size_t                                    m_placement_id{0};

  /*
   * List of cells which have blocks in them. Better to keep a list at the cost
   * of a little extra space, then to recompute a list which will be pretty much
   * the same from timestep to timestep over and over, especially for larger
   * structures.
   */
  std::vector<rmath::vector3z>              m_occupied_cells{};

  /**
   * Specification for all blocks in the structure. Specifically, the
   * specification for the anchor points for all blocks (vertices) and their
   * associated orientations and extents (edges).
   */
  std::unique_ptr<pgds::connectivity_graph> m_spec_graph{nullptr};

  /**
   * We build a map of {block anchor coord -> block spec} so that we have fast
   * lookup of "is this a valid place to put a block" at runtime.
   */
  rdgraph::grid_index_map<rmath::vector3z,
                          repr::block_placement_spec,
                          rmath::vector3z::key_compare> m_placement_map{};

  /**
   * We build a spatial index of block anchor points so that we have fast
   * queries like "what are the closest k points to this one".
   */
  std::unique_ptr<pgds::block_anchor_index> m_anchor_index{nullptr};

  /**
   * \brief Convenience representation of the bounding box+virtual shell around
   * the structure.
   */
  std::unique_ptr<pgrepr::vshell>           m_vshell;

  /**
   * Subtargets within the structure.
   */
  subtarget_vectoro                         m_subtargetso{};
  subtarget_vectorno                        m_subtargetsno{};
  /* clang-format on */


 public:
  RCPPSW_WRAP_DECL(rmath::vector3d, roriginr, const);
  RCPPSW_WRAP_DECL(rmath::vector3z, rorigind, const);
  RCPPSW_WRAP_DECL(rmath::vector3d, voriginr, const);
  RCPPSW_WRAP_DECL(rmath::vector3z, vorigind, const);
};


NS_END(gmt, prism);

#endif /* INCLUDE_PRISM_GMT_SPC_GMT_HPP_ */
