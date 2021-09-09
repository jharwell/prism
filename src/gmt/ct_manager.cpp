/**
 * \file ct_manager.cpp
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
#include "prism/gmt/ct_manager.hpp"

#include "cosm/arena/base_arena_map.hpp"
#include "cosm/repr/config/nests_config.hpp"

#include "prism/gmt/builder_factory.hpp"
#include "prism/gmt/config/gmt_config.hpp"
#include "prism/gmt/config/spct_builder_config.hpp"
#include "prism/gmt/operations/constructability_check.hpp"
#include "prism/gmt/spc_gmt.hpp"
#include "prism/gmt/repr/vshell.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
ct_manager::ct_manager(carena::base_arena_map* const map,
                       cpal::argos_sm_adaptor* const sm,
                       pstv::env_dynamics* const envd)
    : ER_CLIENT_INIT("prism.gmt.ct_manager"),
      m_arena_map(map),
      m_sm(sm),
      m_envd(envd) {}

ct_manager::~ct_manager(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
crepr::config::nests_config ct_manager::initialize(
    const pgconfig::spct_builder_config* builder_config,
    const pgconfig::gmt_config* gmt_config,
    const ctv::config::temporal_penalty_config* placement_penalty_config) {
  ER_INFO("Initializing %zu construction targets",
          gmt_config->spct.size());

  builder_factory factory;
  crepr::config::nests_config nests;

  for (size_t i = 0; i < gmt_config->spct.size(); ++i) {
    ER_INFO("Initializing construction target %zu", i);
    auto target = std::make_unique<pgmt::spc_gmt>(&gmt_config->spct[i],
                                                   m_arena_map,
                                                  rtypes::type_uuid(i));
    if (!construction_feasible(target.get())) {
      ER_WARN("Construction target%s infeasible: will not be built",
              rcppsw::to_string(target->id()).c_str());
      continue;
    }
    auto name = "target" + rcppsw::to_string(target->id()) + "_placement_penalty";

    /* initialize penalty handlers */
    auto handler = std::make_unique<pstv::block_op_penalty_handler>(
        m_arena_map, placement_penalty_config, name);
    m_envd->ct_placement_handler_register(target->id(), std::move(handler));

    /* create associated builder */
    auto builder = factory.create(builder_config->build_src,
                                  builder_config,
                                  target.get(),
                                  m_sm);

    /* create associated nest */
    crepr::config::nest_config nest;
    nest.center = target->vshell()->virt()->rcenter3D().to_2D();
    nest.dims = target->vshell()->virt()->rdims3D().to_2D();
    nests.nests.push_back(nest);

    m_targetsno.push_back(target.get());
    m_targetsro.push_back(target.get());
    m_targetso.push_back(std::move(target));
    m_builderso.push_back(std::move(builder));
    ER_INFO("Initialized construction target%zu", i);
  } /* for(i..) */
  return nests;
} /* initialize() */

void ct_manager::update(const rtypes::timestep& t) {
  for (auto& builder : m_builderso) {
    builder->update(t, m_arena_map->blocks());
  } /* for(&builder..) */
} /* update() */

void ct_manager::reset(void) {
  std::for_each(m_builderso.begin(),
                m_builderso.end(),
                [&](auto& builder) {
                  builder->reset();
                });
  std::for_each(m_targetso.begin(),
                m_targetso.end(),
                [&](auto& target) {
                  target->reset();
                });
} /* reset() */

bool ct_manager::targets_complete(void) const {
  return std::all_of(std::begin(m_targetso),
                     std::end(m_targetso),
                     [&](auto& target) {
                       return target->is_complete();
                     });
} /* targets_complete() */

base_spct_builder*
ct_manager::builder_lookup(const rtypes::type_uuid& target_id) const {
  auto it =
      std::find_if(m_builderso.begin(), m_builderso.end(), [&](auto& builder) {
        return target_id == builder->target_id();
      });
  if (m_builderso.end() != it) {
    return it->get();
  }
  return nullptr;
} /* builder_lookup() */

spc_gmt* ct_manager::target_lookup(const rtypes::type_uuid& id) const {
  auto it = std::find_if(m_targetso.begin(), m_targetso.end(), [&](auto& target) {
    return id == target->id();
  });
  if (m_targetso.end() != it) {
    return it->get();
  }
  return nullptr;
} /* target_lookup() */

bool ct_manager::construction_feasible(const spc_gmt* target) const {
  if (!pgops::constructability_check()(target->spec(),
                                       target->vshell(),
                                       target->orientation())) {
    ER_WARN("Construction target%s invalid",
            rcppsw::to_string(target->id()).c_str());
    return false;
  }
  for (const auto& existing_t : m_targetso) {
    if (existing_t->xrspan().overlaps_with(target->xrspan()) &&
        existing_t->yrspan().overlaps_with(target->yrspan())) {
      ER_WARN("Construction target%s overlaps with target%s",
              rcppsw::to_string(target->id()).c_str(),
              rcppsw::to_string(existing_t->id()).c_str());
      return false;
    }
  } /* for(..&existing_t) */
  return true;
} /* construction_feasible() */

NS_END(gmt, prism);
