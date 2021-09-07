/**
 * \file structure3D.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/structure/structure3D.hpp"

#include <algorithm>
#include <boost/variant/static_visitor.hpp>

#include "cosm/arena/base_arena_map.hpp"

#include "silicon/structure/operations/block_placement_validate.hpp"
#include "silicon/structure/subtarget.hpp"
#include "silicon/algorithm/constants.hpp"
#include "silicon/structure/ds/connectivity_graph.hpp"
#include "silicon/structure/ds/block_anchor_index.hpp"
#include "silicon/structure/repr/vshell.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
structure3D::structure3D(const config::structure3D_config* config,
                         const carena::base_arena_map* map,
                         size_t id)
    : ER_CLIENT_INIT("silicon.structure.structure3D"),
      mc_id(id),
      mc_block_unit_dim(std::min(map->blocks()[0]->rdim2D().x(),
                                 map->blocks()[0]->rdim2D().y())),
      mc_unit_dim_factor(unit_dim_factor_calc(map)),
      mc_arena_grid_res(map->grid_resolution()),
      mc_config(*config),
      m_spec_graph(std::make_unique<ssds::connectivity_graph>(ssds::connectivity_graph::from_file(mc_config.graphml))),
      m_vshell(std::make_unique<ssrepr::vshell>(&map->decoratee(),
                                                config->anchor,
                                                config->bounding_box,
                                                mc_block_unit_dim)),
      m_subtargetso(subtargetso_init()),
      m_subtargetsno(subtargetsno_init()) {

  ER_INFO("Structure%s: vorigin=%s/%s,rorigin=%s/%s",
          rcppsw::to_string(mc_id).c_str(),
          rcppsw::to_string(voriginr()).c_str(),
          rcppsw::to_string(vorigind()).c_str(),
          rcppsw::to_string(roriginr()).c_str(),
          rcppsw::to_string(rorigind()).c_str());

  ER_INFO("Structure%s: rxrspan=%s,vxrspan=%s ryrspanr=%s,vyrspan=%s",
          rcppsw::to_string(mc_id).c_str(),
          rcppsw::to_string(vshell()->xrspan(true)).c_str(),
          rcppsw::to_string(vshell()->xrspan(false)).c_str(),
          rcppsw::to_string(vshell()->yrspan(true)).c_str(),
          rcppsw::to_string(vshell()->yrspan(false)).c_str());

  /* initialize data structures */
  ds_init();
}

structure3D::~structure3D(void) = default;

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/
size_t
structure3D::unit_dim_factor_calc(const carena::base_arena_map* map) const {
  ER_ASSERT(std::fmod(mc_block_unit_dim.v(), map->grid_resolution().v()) <=
            std::numeric_limits<double>::epsilon(),
            "Block unit dimension (%f) not a multiple of arena grid resolution "
            "(%f)",
            mc_block_unit_dim.v(),
            map->grid_resolution().v());
  return static_cast<size_t>(mc_block_unit_dim.v() / map->grid_resolution().v());
} /* unit_dim_factor_calc() */

void structure3D::ds_init(void) {
  ER_DEBUG("Initialize data structures for structure%d: shell_size=%zu",
           mc_id.v(),
           vshell()->sh_sized());

  for (size_t i = 0; i < xdsize(); ++i) {
    for (size_t j = 0; j < ydsize(); ++j) {
      for (size_t k = 0; k < zdsize(); ++k) {
        rmath::vector3z c(i, j, k);
        if (auto vd = m_spec_graph->find(c)) {
          auto spec = m_spec_graph->to_spec(*vd);
          m_placement_map[c] = ssrepr::block_placement_spec{spec, nullptr};
          m_anchor_index->insert(*vd, rds::make_rtree_point(c));
        }
      } /* for(k..) */
    } /* for(j..) */
  } /* for(i..) */
  ER_DEBUG("Data structure initialization for structure%d complete", mc_id.v());
} /* ds_init() */

structure3D::subtarget_vectoro structure3D::subtargetso_init(void) const {
  subtarget_vectoro ret;
  if (rmath::radians::kZERO == orientation() ||
      rmath::radians::kPI == orientation()) {
    for (size_t j = 0; j < ydsize() / saconstants::kCT_SUBTARGET_WIDTH_CELLS; ++j) {
      ER_INFO("Calculating subtarget %zu along Y slice axis", j);
      ret.push_back(std::make_unique<subtarget>(this, j));
    } /* for(j..) */
  } else if (rmath::radians::kPI_OVER_TWO == orientation() ||
             rmath::radians::kTHREE_PI_OVER_TWO == orientation()) {
    for (size_t i = 0; i < xdsize() / saconstants::kCT_SUBTARGET_WIDTH_CELLS; ++i) {
      ER_INFO("Calculating subtarget %zu along X slice axis", i);
      ret.push_back(std::make_unique<subtarget>(this, i));
    } /* for(i..) */
  }
  return ret;
} /* subtargetso_init() */

structure3D::subtarget_vectorno structure3D::subtargetsno_init(void) const {
  subtarget_vectorno ret;
  std::transform(m_subtargetso.begin(),
                 m_subtargetso.end(),
                 std::back_inserter(ret),
                 [&](const auto& s) { return s.get(); });
  return ret;
} /* subtargetsno_init() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool structure3D::block_placement_valid(const crepr::block3D_variantro& block,
                                        const srepr::placement_intent& intent) {
  auto validator = operations::block_placement_validate(this, intent);
  return boost::apply_visitor(validator, block);
} /* block_placement_valid() */

bool structure3D::spec_exists(const crepr::base_block3D* const query) const {
  auto coord = ssrepr::ct_coord{ query->danchor3D(),
                               ssrepr::ct_coord::relativity::ekVORIGIN,
                               this };
  return nullptr != spec_retrieve(coord);
} /* spec_exists() */

bool structure3D::contains(const rmath::vector2d& loc,
                           bool include_virtual) const {
  return vshell()->xrspan(include_virtual).contains(loc.x()) &&
      vshell()->yrspan(include_virtual).contains(loc.y());
} /* contains() */

void structure3D::block_add(std::unique_ptr<crepr::base_block3D> block) {
  RCSW_UNUSED rtypes::type_uuid id = block->id();
  m_occupied_cells.push_back((block->danchor3D() - vorigind()) /
                             mc_unit_dim_factor);

  /* add block to spec graph so robots can see it in their LOS */
  m_spec_graph->to_spec(m_spec_graph->find(block->danchor3D()))->block = block.get();

  /* add block to map of block placements */
  m_placement_map[block->danchor3D() - vorigind()].block = std::move(block);

  ++m_placed;
  ++m_placed_since_reset;
  ER_INFO("Added block%d to structure (%zu total)", id.v(), m_placed);
} /* block_add() */

const ssrepr::block_anchor_spec*
structure3D::spec_retrieve(const ssrepr::ct_coord& coord) const {
  /*
   * If the coordinates are relative to the virtual origin, we have to translate
   * them to be relative to the REAL origin, as only cells that are ACTUALLY
   * part of the structure's bounding box can have specs.
   */
  auto rcoord = coord.to_real();
  auto it = m_placement_map.find(rcoord.offset());
  if (m_placement_map.end() != it) {
    return it->second.spec;
  }
  return nullptr;
} /* spec_retrieve() */

rmath::vector3d structure3D::anchor_loc_abs(const ssrepr::ct_coord& anchor) const {
  return voriginr() +
         rmath::zvec2dvec(anchor.to_virtual().offset()) *
      unit_dim_factor() *
      mc_arena_grid_res.v();
} /* anchor_loc_abs() */

subtarget* structure3D::parent_subtarget(const ssrepr::ct_coord& coord) {
  size_t index = 0;

  auto rcoord = coord.to_real();
  if (rmath::radians::kZERO == orientation() ||
      rmath::radians::kPI == orientation()) {
    index = rcoord.offset().y() / saconstants::kCT_SUBTARGET_WIDTH_CELLS;
  } else if (rmath::radians::kPI_OVER_TWO == orientation() ||
             rmath::radians::kTHREE_PI_OVER_TWO == orientation()) {
    index = rcoord.offset().x() / saconstants::kCT_SUBTARGET_WIDTH_CELLS;
  }
  ER_ASSERT(
      index <= m_subtargetsno.size(), "Bad index %zu: no such subtarget", index);
  return m_subtargetsno[index];
} /* parent_subtarget() */

void structure3D::reset(void) {
  m_placement_id = 0;
  reset_metrics();

  /* remove all placed blocks */
  for (auto &e : m_placement_map) {
    e.second.block = nullptr;
  } /* for(&e..) */
} /* reset() */

void structure3D::reset_metrics(void) {
  m_placed_since_reset = 0;
  for (auto* t : subtargets()) {
    t->reset_metrics();
  } /* for(*t..) */
} /* reset_metrics() */

/*******************************************************************************
 * Progress Metrics
 ******************************************************************************/
size_t structure3D::n_total_placed(void) const { return m_placed; }
size_t structure3D::n_interval_placed(void) const { return m_placed_since_reset; }

size_t structure3D::manifest_size(void) const { return m_spec_graph->n_vertices(); }

bool structure3D::is_complete(void) const {
  /* basic sanity checks */
  return n_total_placed() == manifest_size();
} /* is_complete() */

bool structure3D::post_completion_check(void) const {
  ER_CHECK(n_total_placed() == manifest_size(),
           "Not all blocks placed: %zu != %zu",
           n_total_placed(),
           manifest_size());

  /*
   * \todo This is inefficent and should be made better at some point.
   */
  for (size_t i = 0; i < xdsize(); ++i) {
    for (size_t j = 0; j < ydsize(); ++j) {
      for (size_t k = 0; k < zdsize(); ++k) {
        auto coord = ssrepr::ct_coord{ rmath::vector3z{ i, j, k },
                                     ssrepr::ct_coord::relativity::ekVORIGIN,
                                     this };
        const auto* spec = spec_retrieve(coord);

        if (nullptr == spec) {
          continue;
        }
        const auto& placement = m_placement_map.find(coord.to_real().offset());
        ER_CHECK(nullptr != placement->second.block,
                 "Cell@%s should be the anchor for a block",
                 rcppsw::to_string(coord.to_real().offset()).c_str());
      } /* for(k..) */
    } /* for(j..) */
  } /* for(i..) */

  return true;

error:
  return false;
} /* post_completion_check() */

RCPPSW_WRAP_DEF(structure3D, roriginr, *m_vshell, const);
RCPPSW_WRAP_DEF(structure3D, rorigind, *m_vshell, const);
RCPPSW_WRAP_DEF(structure3D, voriginr, *m_vshell, const);
RCPPSW_WRAP_DEF(structure3D, vorigind, *m_vshell, const);

/* entity3D overrides */
RCPPSW_WRAP_DEF_OVERRIDE(structure3D, rcenter3D, *m_vshell->real(), const);
RCPPSW_WRAP_DEF_OVERRIDE(structure3D, ranchor3D, *m_vshell->real(), const);

RCPPSW_WRAP_DEF_OVERRIDE(structure3D, dcenter3D, *m_vshell->real(), const);
RCPPSW_WRAP_DEF_OVERRIDE(structure3D, danchor3D, *m_vshell->real(), const);

RCPPSW_WRAP_DEF_OVERRIDE(structure3D, xrspan, *m_vshell->real(), const);
RCPPSW_WRAP_DEF_OVERRIDE(structure3D, yrspan, *m_vshell->real(), const);
RCPPSW_WRAP_DEF_OVERRIDE(structure3D, zrspan, *m_vshell->real(), const);

RCPPSW_WRAP_DEF_OVERRIDE(structure3D, xrsize, *m_vshell->real(), const);
RCPPSW_WRAP_DEF_OVERRIDE(structure3D, yrsize, *m_vshell->real(), const);
RCPPSW_WRAP_DEF_OVERRIDE(structure3D, zrsize, *m_vshell->real(), const);

RCPPSW_WRAP_DEF_OVERRIDE(structure3D, xdspan, *m_vshell->real(), const);
RCPPSW_WRAP_DEF_OVERRIDE(structure3D, ydspan, *m_vshell->real(), const);
RCPPSW_WRAP_DEF_OVERRIDE(structure3D, zdspan, *m_vshell->real(), const);

RCPPSW_WRAP_DEF_OVERRIDE(structure3D, xdsize, *m_vshell->real(), const);
RCPPSW_WRAP_DEF_OVERRIDE(structure3D, ydsize, *m_vshell->real(), const);
RCPPSW_WRAP_DEF_OVERRIDE(structure3D, zdsize, *m_vshell->real(), const);

NS_END(structure, silicon);
