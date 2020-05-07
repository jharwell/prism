/**
 * \file ct_manager.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_CT_MANAGER_HPP_
#define INCLUDE_SILICON_STRUCTURE_CT_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <memory>

#include "rcppsw/types/timestep.hpp"
#include "rcppsw/control/config/waveform_config.hpp"
#include "rcppsw/er/client.hpp"

#include "silicon/silicon.hpp"
#include "silicon/ds/construct_target_vector.hpp"
#include "silicon/support/tv/env_dynamics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace silicon::structure {
namespace config {
struct structure3D_builder_config;
struct construct_targets_config;
} /* namespace config */

class structure3D_builder;
class structure3D;
} /* namespace silicon::structure */

namespace cosm::arena {
class base_arena_map;
} /* namespace cosm::arena */

namespace cosm::pal {
class argos_sm_adaptor;
} /* namespace cosm::pal */

NS_START(silicon, structure);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ct_manager
 * \ingroup structure
 *
 * \brief Manager for the \ref structure::structure3D objects and their
 * associated \ref structure::structure3D_builder objects, as a convenience
 * wrapper allowing for a clean interface for large numbers of robots
 * interacting with multiple ct simultaneously in the arena.
 */
class ct_manager : public rer::client<ct_manager> {
 public:
  ct_manager(carena::base_arena_map* map,
                     cpal::argos_sm_adaptor* sm,
                     sstv::env_dynamics* envd);

  ~ct_manager(void) override;

  /* Not copy constructable/assignable by default */
  ct_manager(const ct_manager&) = delete;
  const ct_manager& operator=(const ct_manager&) = delete;

  /**
   * \brief Initialize the manager.
   *
   * \param builder_config Config for the \ref structure::structure3D_builder
   *                       (same config for builders for all ct).
   * \param targets_config Config for the \ref structure::structure3D objects;
   *                       (unique config for each structure).
   * \param placement_penalty_config Config for the block placement penalty
   *                                 (same config for all all lanes for all
   *                                 ct).
   */
  void init(const ssconfig::structure3D_builder_config* builder_config,
            const ssconfig::construct_targets_config* targets_config,
            const rct::config::waveform_config* const placement_penalty_config);

  /**
   * \brief The vector of all targets to be built in the arena.
   */

  const ds::construct_target_vectorno& targets(void) const {
    return m_targetsno;
  }

  const support::tv::block_op_penalty_handler* placement_handler(
      const rtypes::type_uuid& target_id) const {
    return m_envd->ct_penalty_handler(sstv::block_op_src::ekCT_BLOCK_MANIP,
                                       target_id);
  }
  support::tv::block_op_penalty_handler* placement_handler(
      const rtypes::type_uuid& target_id) {
    return m_envd->ct_penalty_handler(sstv::block_op_src::ekCT_BLOCK_MANIP,
                                       target_id);
  }

  const structure3D_builder* builder(const rtypes::type_uuid& target_id) const {
    return builder_lookup(target_id);
  }
  structure3D_builder* builder(const rtypes::type_uuid& target_id) {
    return const_cast<const ct_manager*>(this)->builder_lookup(target_id);
  }
  const structure3D* target(const rtypes::type_uuid& id) const {
    return target_lookup(id);
  }
  structure3D* target(const rtypes::type_uuid& id) {
    return const_cast<const ct_manager*>(this)->target_lookup(id);
  }

  /**
   * \brief Update all construction targets, builders, on a given timestep.
   *
   * This includes:
   *
   * - Static structure building
   */
  void update(const rtypes::timestep& t);

  /**
   * \brief Reset all construction targets and their builders
   */
  void reset(void);

 private:
  using builders_vectoro_type =
      std::vector<std::unique_ptr<structure::structure3D_builder>>;

  structure3D_builder* builder_lookup(const rtypes::type_uuid& target_id) const;
  structure3D* target_lookup(const rtypes::type_uuid& id) const;

  /* clang-format off */
  carena::base_arena_map*               m_arena_map;
  cpal::argos_sm_adaptor*               m_sm;
  support::tv::env_dynamics*            m_envd;
  ds::construct_target_vectoro          m_targetso{};
  ds::construct_target_vectorno         m_targetsno{};
  builders_vectoro_type                 m_builderso{};
  /* clang-format on */
};

NS_END(structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_CT_MANAGER_HPP_ */
