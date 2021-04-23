/**
 * \file ct_manager.cpp
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
#include "silicon/structure/ct_manager.hpp"

#include "cosm/arena/base_arena_map.hpp"

#include "silicon/structure/builder_factory.hpp"
#include "silicon/structure/config/construct_targets_config.hpp"
#include "silicon/structure/config/structure3D_builder_config.hpp"
#include "silicon/structure/operations/validate_spec.hpp"
#include "silicon/structure/structure3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
ct_manager::ct_manager(carena::base_arena_map* const map,
                       cpal::argos_sm_adaptor* const sm,
                       sstv::env_dynamics* const envd)
    : ER_CLIENT_INIT("silicon.structure.ct_manager"),
      m_arena_map(map),
      m_sm(sm),
      m_envd(envd) {}

ct_manager::~ct_manager(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void ct_manager::init(
    const ssconfig::structure3D_builder_config* builder_config,
    const ssconfig::construct_targets_config* targets_config,
    const ctv::config::temporal_penalty_config* placement_penalty_config) {
  ER_INFO("Initializing %zu construction targets",
          targets_config->targets.size());

  builder_factory factory;

  for (size_t i = 0; i < targets_config->targets.size(); ++i) {
    ER_INFO("Initializing construction target %zu", i);
    auto target = std::make_unique<sstructure::structure3D>(
        &targets_config->targets[i], m_arena_map, i);
    if (!construction_feasible(target.get())) {
      ER_WARN("Construction target%s infeasible: will not be built",
              rcppsw::to_string(target->id()).c_str());
      continue;
    }
    auto name = "target" + rcppsw::to_string(target->id()) + "_placement_penalty";
    auto handler = std::make_unique<sstv::block_op_penalty_handler>(
        m_arena_map, placement_penalty_config, name);
    m_envd->ct_placement_handler_register(target->id(), std::move(handler));
    auto builder = factory.create(
        builder_config->build_src, builder_config, target.get(), m_sm);
    m_targetsno.push_back(target.get());
    m_targetsro.push_back(target.get());
    m_targetso.push_back(std::move(target));
    m_builderso.push_back(std::move(builder));
    ER_INFO("Initialized construction target%zu", i);
  } /* for(i..) */
} /* init() */

void ct_manager::update(const rtypes::timestep& t) {
  for (auto& builder : m_builderso) {
    builder->update(t, m_arena_map->blocks());
  } /* for(&builder..) */
} /* update() */

void ct_manager::reset(void) {
  for (auto& builder : m_builderso) {
    builder->reset();
  } /* for(&builder..) */

  for (auto& target : m_targetso) {
    target->reset();
  } /* for(&target..) */
} /* reset() */

base_structure3D_builder*
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

structure3D* ct_manager::target_lookup(const rtypes::type_uuid& id) const {
  auto it = std::find_if(m_targetso.begin(), m_targetso.end(), [&](auto& target) {
    return id == target->id();
  });
  if (m_targetso.end() != it) {
    return it->get();
  }
  return nullptr;
} /* target_lookup() */

bool ct_manager::construction_feasible(const structure3D* target) const {
  if (!ssops::validate_spec(target)()) {
    ER_WARN("Construction target%s invalid",
            rcppsw::to_string(target->id()).c_str());
    return false;
  }
  for (const auto& existing_t : m_targetso) {
    if (existing_t->xranger().overlaps_with(target->xranger()) &&
        existing_t->yranger().overlaps_with(target->yranger())) {
      ER_WARN("Construction target%s overlaps with target%s",
              rcppsw::to_string(target->id()).c_str(),
              rcppsw::to_string(existing_t->id()).c_str());
      return false;
    }
  } /* for(..&existing_t) */
  return true;
} /* construction_feasible() */

NS_END(structure, silicon);
