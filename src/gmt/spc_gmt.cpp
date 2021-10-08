/**
 * \file spc_gmt.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/gmt/spc_gmt.hpp"

#include <algorithm>
#include <boost/variant/static_visitor.hpp>

#include "cosm/arena/base_arena_map.hpp"
#include "cosm/repr/sim_block3D.hpp"

#include "prism/gmt/operations/block_placement_validate.hpp"
#include "prism/gmt/subtarget.hpp"
#include "prism/properties/algorithm.hpp"
#include "prism/gmt/ds/connectivity_graph.hpp"
#include "prism/gmt/ds/block_anchor_index.hpp"
#include "prism/gmt/repr/vshell.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
spc_gmt::spc_gmt(const config::spct_config* config,
                 const carena::base_arena_map* map,
                 const rtypes::type_uuid& id)
    : ER_CLIENT_INIT("prism.gmt.spc_gmt"),
      entity3D(id,
               rmath::zvec2dvec(config->bounding_box,
                                block_unit_dim_calc(map->blocks()).v()),
               config->anchor,
               block_unit_dim_calc(map->blocks())),
      mc_block_unit_dim(block_unit_dim_calc(map->blocks())),
      mc_unit_dim_factor(unit_dim_factor_calc(map)),
      mc_arena_grid_res(map->grid_resolution()),
      mc_config(*config),
      m_spec_graph(std::make_unique<pgds::connectivity_graph>(pgds::connectivity_graph::from_file(mc_config.graphml))),
      m_anchor_index(std::make_unique<pgds::block_anchor_index>()),
      m_vshell(std::make_unique<pgrepr::vshell>(&map->decoratee(),
                                                config->anchor,
                                                config->bounding_box,
                                                mc_block_unit_dim)),
      m_subtargetso(subtargetso_init()),
      m_subtargetsno(subtargetsno_init()) {
  ER_INFO("Structure%s: vorigin=%s/%s,rorigin=%s/%s",
          rcppsw::to_string(this->id()).c_str(),
          rcppsw::to_string(voriginr()).c_str(),
          rcppsw::to_string(vorigind()).c_str(),
          rcppsw::to_string(roriginr()).c_str(),
          rcppsw::to_string(rorigind()).c_str());

  ER_INFO("Structure%s: rxrspan=%s,vxrspan=%s ryrspanr=%s,vyrspan=%s",
          rcppsw::to_string(this->id()).c_str(),
          rcppsw::to_string(vshell()->xrspan(true)).c_str(),
          rcppsw::to_string(vshell()->xrspan(false)).c_str(),
          rcppsw::to_string(vshell()->yrspan(true)).c_str(),
          rcppsw::to_string(vshell()->yrspan(false)).c_str());

  /* initialize data structures */
  ds_init();
}

spc_gmt::~spc_gmt(void) = default;

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/
rtypes::spatial_dist spc_gmt::block_unit_dim_calc(
    const cds::block3D_vectorno& blocks) {
  rtypes::spatial_dist ret{std::numeric_limits<double>::max()};
  for (auto &b : blocks) {
    ret = rtypes::spatial_dist(std::min({ret.v(),
                                         b->rdims2D().x(),
                                         b->rdims2D().y()}));
  } /* for(&b..) */
  return ret;
} /* block_unit_dim_calc() */

size_t
spc_gmt::unit_dim_factor_calc(const carena::base_arena_map* map) const {
  ER_ASSERT(std::fmod(mc_block_unit_dim.v(), map->grid_resolution().v()) <=
            std::numeric_limits<double>::epsilon(),
            "Block unit dimension (%f) not a multiple of arena grid resolution "
            "(%f)",
            mc_block_unit_dim.v(),
            map->grid_resolution().v());
  return static_cast<size_t>(mc_block_unit_dim.v() / map->grid_resolution().v());
} /* unit_dim_factor_calc() */

void spc_gmt::ds_init(void) {
  ER_DEBUG("Initialize data structures for structure%d: shell_size=%zu",
           id().v(),
           vshell()->sh_sized());

  for (size_t i = 0; i < xdsize(); ++i) {
    for (size_t j = 0; j < ydsize(); ++j) {
      for (size_t k = 0; k < zdsize(); ++k) {
        rmath::vector3z c(i, j, k);
        if (auto vd = m_spec_graph->find(c)) {
          auto spec = m_spec_graph->to_spec(*vd);
          m_placement_map[c] = pgrepr::block_placement_spec{spec, nullptr};
          m_anchor_index->insert(*vd, rds::make_rtree_point(c));
          ER_TRACE("Initialize cell@%s", rcppsw::to_string(c).c_str());
        }
      } /* for(k..) */
    } /* for(j..) */
  } /* for(i..) */
  ER_DEBUG("Data structure initialization for structure%d complete", id().v());
} /* ds_init() */

spc_gmt::subtarget_vectoro spc_gmt::subtargetso_init(void) const {
  subtarget_vectoro ret;
  if (rmath::radians::kZERO == orientation() ||
      rmath::radians::kPI == orientation()) {
    for (size_t j = 0; j < ydsize() / pproperties::algorithm::kCT_SUBTARGET_WIDTH_CELLS; ++j) {
      ER_INFO("Calculating subtarget %zu along Y slice axis", j);
      ret.push_back(std::make_unique<subtarget>(this, j));
    } /* for(j..) */
  } else if (rmath::radians::kPI_OVER_TWO == orientation() ||
             rmath::radians::kTHREE_PI_OVER_TWO == orientation()) {
    for (size_t i = 0; i < xdsize() / pproperties::algorithm::kCT_SUBTARGET_WIDTH_CELLS; ++i) {
      ER_INFO("Calculating subtarget %zu along X slice axis", i);
      ret.push_back(std::make_unique<subtarget>(this, i));
    } /* for(i..) */
  }
  return ret;
} /* subtargetso_init() */

spc_gmt::subtarget_vectorno spc_gmt::subtargetsno_init(void) const {
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
bool spc_gmt::block_placement_valid(const crepr::block3D_variantro& block,
                                        const prepr::placement_intent& intent) {
  auto validator = operations::block_placement_validate(this, intent);
  return std::visit(validator, block);
} /* block_placement_valid() */

bool spc_gmt::spec_exists(const crepr::base_block3D* const query) const {
  auto coord = pgrepr::ct_coord{ query->danchor3D(),
                               pgrepr::ct_coord::relativity::ekVORIGIN,
                               this };
  return nullptr != spec_retrieve(coord);
} /* spec_exists() */

bool spc_gmt::contains(const rmath::vector2d& loc,
                           bool include_virtual) const {
  return vshell()->xrspan(include_virtual).contains(loc.x()) &&
      vshell()->yrspan(include_virtual).contains(loc.y());
} /* contains() */

void spc_gmt::block_add(std::unique_ptr<crepr::base_block3D> block) {
  RCPPSW_UNUSED auto id = block->id();
  RCPPSW_UNUSED auto graph_coord = block->danchor3D() - rorigind();

  /* add block to spec graph so robots can see it in their LOS */
  auto vd = m_spec_graph->find(graph_coord);
  ER_ASSERT(boost::none != vd, "Spec for block to add@%s is NULL",
            rcppsw::to_string(graph_coord).c_str());
  m_spec_graph->to_spec(vd)->block = block.get();

  /* add block to map of block placements */
  m_placement_map[graph_coord].block = std::move(block);

  /* update metrics */
  ++m_placed;
  ++m_placed_since_reset;
  m_occupied_cells.push_back(graph_coord / mc_unit_dim_factor);

  ER_INFO("Added block%d to structure (%zu total)", id.v(), m_placed);
} /* block_add() */

const pgrepr::block_anchor_spec*
spc_gmt::spec_retrieve(const pgrepr::ct_coord& coord) const {
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

rmath::vector3d spc_gmt::anchor_loc_abs(const pgrepr::ct_coord& anchor) const {
  return voriginr() +
         rmath::zvec2dvec(anchor.to_virtual().offset()) *
      unit_dim_factor() *
      mc_arena_grid_res.v();
} /* anchor_loc_abs() */

subtarget* spc_gmt::parent_subtarget(const pgrepr::ct_coord& coord) {
  size_t index = 0;

  auto rcoord = coord.to_real();
  if (rmath::radians::kZERO == orientation() ||
      rmath::radians::kPI == orientation()) {
    index = rcoord.offset().y() / ppalgorithm::kCT_SUBTARGET_WIDTH_CELLS;
  } else if (rmath::radians::kPI_OVER_TWO == orientation() ||
             rmath::radians::kTHREE_PI_OVER_TWO == orientation()) {
    index = rcoord.offset().x() / ppalgorithm::kCT_SUBTARGET_WIDTH_CELLS;
  }
  ER_ASSERT(
      index <= m_subtargetsno.size(), "Bad index %zu: no such subtarget", index);
  return m_subtargetsno[index];
} /* parent_subtarget() */

void spc_gmt::reset(void) {
  m_placement_id = 0;
  reset_metrics();

  /* remove all placed blocks */
  for (auto &e : m_placement_map) {
    e.second.block = nullptr;
  } /* for(&e..) */
} /* reset() */

void spc_gmt::reset_metrics(void) {
  m_placed_since_reset = 0;
  for (auto* t : subtargets()) {
    t->reset_metrics();
  } /* for(*t..) */
} /* reset_metrics() */

/*******************************************************************************
 * Progress Metrics
 ******************************************************************************/
size_t spc_gmt::n_total_placed(void) const { return m_placed; }
size_t spc_gmt::n_interval_placed(void) const { return m_placed_since_reset; }

size_t spc_gmt::manifest_size(void) const { return m_spec_graph->n_vertices(); }

bool spc_gmt::is_complete(void) const {
  /* basic sanity checks */
  return n_total_placed() == manifest_size();
} /* is_complete() */

bool spc_gmt::post_completion_check(void) const {
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
        auto coord = pgrepr::ct_coord{ rmath::vector3z{ i, j, k },
                                     pgrepr::ct_coord::relativity::ekVORIGIN,
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

RCPPSW_WRAP_DEF(spc_gmt, roriginr, *m_vshell, const);
RCPPSW_WRAP_DEF(spc_gmt, rorigind, *m_vshell, const);
RCPPSW_WRAP_DEF(spc_gmt, voriginr, *m_vshell, const);
RCPPSW_WRAP_DEF(spc_gmt, vorigind, *m_vshell, const);

NS_END(gmt, prism);
