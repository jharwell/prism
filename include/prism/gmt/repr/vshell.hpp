/**
 * \file vshell.hpp
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

#ifndef INCLUDE_PRISM_GMT_REPR_VSHELL_HPP_
#define INCLUDE_PRISM_GMT_REPR_VSHELL_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/repr/gridQ3D_view_entity.hpp"
#include "cosm/arena/ds/arena_grid.hpp"

#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class vshell
 * \ingroup gmt repr
 *
 * \brief Representation of the virtual 3D shell around a \ref spc_gmt. The
 * shell does not really exist, so robots can't interact with it.
 *
 * This class assumes that the resolution of the 3D grid for the structure is
 * the same as the arena (i.e., a block that is 0.4x0.4 in the arena is the same
 * size when placed on the structure).
 */
class vshell : public rer::client<vshell> {
 public:
  /**
   * \brief The size of the virtual shell, in units of # cells;
   */
  static constexpr const size_t kWIDTH_CELLS = 2;

  using grid_view_entity_type = crepr::gridQ3D_view_entity<cads::arena_grid,
                                                           cads::arena_grid::const_view>;
  using grid_view_type = typename grid_view_entity_type::grid_view_type;

  /**
   * \brief Get the size of the virtual shell which surrounds the target, in
   * units of # cells;
   */
  static size_t sh_sized(void) { return kWIDTH_CELLS; }

  vshell(const carena::ds::arena_grid* grid,
         const rmath::vector3d& ct_origin,
         const rmath::vector3z& ct_bb,
         const rtypes::spatial_dist& unit_dim);

  /* Not move/copy constructable/assignable by default */
  vshell(const vshell&) = delete;
  vshell& operator=(const vshell&) = delete;
  vshell(vshell&&) = delete;
  vshell& operator=(vshell&&) = delete;

  const rtypes::spatial_dist& unit_dim(void) const { return mc_unit_dim; }

  /**
   * \brief Get the size of the virtual shell, in spatial distance;
   */
  rtypes::spatial_dist sh_sizer(void) const { return mc_unit_dim * kWIDTH_CELLS; }

  rmath::ranged xrspan(bool include_virt = false) const {
    if (include_virt) {
      return rmath::ranged(voriginr().x(),
                           voriginr().x() + m_outer.xrsize().v());
    } else {
      return rmath::ranged(roriginr().x(),
                           roriginr().x() + m_inner.xrsize().v());
    }
  }
  rmath::ranged yrspan(bool include_virt = false) const {
    if (include_virt) {
      return rmath::ranged(voriginr().y(),
                           voriginr().y() + m_outer.yrsize().v());
    } else {
      return rmath::ranged(roriginr().y(),
                           roriginr().y() + m_inner.yrsize().v());
    }
  }
  rmath::ranged zrspan(void) const {
    return rmath::ranged(roriginr().z(),
                         roriginr().z() + m_inner.zrsize().v());
  }

  rmath::rangez xdspan(bool include_virt = false) const {
    if (include_virt) {
      return rmath::rangez(vorigind().x(),
                           vorigind().x() + m_outer.xdsize());
    } else {
      return rmath::rangez(rorigind().x(),
                           rorigind().x() + m_inner.xdsize());
    }
  }
  rmath::rangez ydspan(bool include_virt = false) const {
    if (include_virt) {
      return rmath::rangez(vorigind().y(),
                           vorigind().y() + m_outer.ydsize());
    } else {
      return rmath::rangez(rorigind().y(),
                           rorigind().y() + m_inner.ydsize());
    }
  }
  rmath::rangez zdspan(void) const {
    return rmath::rangez(rorigind().z(),
                         rorigind().z() + m_inner.zdsize());
  }

  const grid_view_entity_type* real(void) const { return &m_inner; }
  const grid_view_entity_type* virt(void) const { return &m_outer; }

 private:
  /**
   * \brief Perform initialization sanity checks post-hoc:
   *
   * - The real/virtual origin coordinates (X,Y) are a multiple of the 3D grid
   *   resolution.
   */
  bool initialization_checks(void) const;

  grid_view_entity_type outer_init(
      const carena::ds::arena_grid* grid,
      const rmath::vector3d& ct_origin,
      const rmath::vector3z& ct_bb) const;

  grid_view_entity_type inner_init(
      const carena::ds::arena_grid* grid,
      const rmath::vector3d& ct_origin,
      const rmath::vector3z& ct_bb) const;

    /* clang-format off */
  const rtypes::spatial_dist mc_unit_dim;

  grid_view_entity_type      m_outer;
  grid_view_entity_type      m_inner;
  /* clang-format on */

 public:
  /**
   * \brief Get the real arena coordinates of the origin (LL corner) of the
   * outer part of the virtual shell which surrounds the a \ref spc_gmt.
   */
  RCPPSW_WRAP_DECLDEF_AS(ranchor3D, m_outer, voriginr, const);

  /**
   * \brief Get the real arena coordinates of the origin (LL corner) of the
   * inner part of a the virtual shell which surrounds a \ref spc_gmt; this
   * is the origin of the structure.
   */
  RCPPSW_WRAP_DECLDEF_AS(ranchor3D, m_inner, roriginr, const);

  /**
   * \brief Get the discrete arena coordinates of the origin (LL corner) of the
   * outer part of the virtual shell which surrounds a \ref spc_gmt.
   */
  RCPPSW_WRAP_DECLDEF_AS(danchor3D, m_outer, vorigind, const);

  /**
   * \brief Get the discrete arena coordinates of the origin (LL corner) of the
   * inner part of a the virtual shell which surrounds a \ref spc_gmt; this
   * is the origin of the structure.
   */
  RCPPSW_WRAP_DECLDEF_AS(danchor3D, m_inner, rorigind, const);
};

NS_END(repr, gmt, prism);

#endif /* INCLUDE_PRISM_GMT_REPR_VSHELL_HPP_ */
