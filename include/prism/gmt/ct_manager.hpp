/**
 * \file ct_manager.hpp
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

#ifndef INCLUDE_PRISM_GMT_CT_MANAGER_HPP_
#define INCLUDE_PRISM_GMT_CT_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <vector>

#include "rcppsw/control/config/waveform_config.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/types/timestep.hpp"

#include "prism/ds/ct_vector.hpp"
#include "prism/prism.hpp"
#include "prism/support/tv/env_dynamics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace prism::gmt {
namespace config {
struct spct_builder_config;
struct gmt_config;
} /* namespace config */

class base_spct_builder;
class spc_gmt;
} /* namespace prism::gmt */

namespace cosm::arena {
class base_arena_map;
} /* namespace cosm::arena */

namespace cosm::pal {
class argos_sm_adaptor;
} /* namespace cosm::pal */

NS_START(prism, gmt);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ct_manager
 * \ingroup gmt
 *
 * \brief Manager for the \ref gmt::spc_gmt objects and their
 * associated \ref gmt::spct_builder objects, as a convenience
 * wrapper allowing for a clean interface for large numbers of robots
 * interacting with multiple ct simultaneously in the arena.
 */
class ct_manager : public rer::client<ct_manager> {
 public:
  ct_manager(carena::base_arena_map* map,
             cpal::argos_sm_adaptor* sm,
             pstv::env_dynamics* envd);

  ~ct_manager(void) override;

  /* Not copy constructable/assignable by default */
  ct_manager(const ct_manager&) = delete;
  const ct_manager& operator=(const ct_manager&) = delete;

  /**
   * \brief Initialize the manager.
   *
   * \param builder_config Config for the \ref gmt::spct_builder
   *                       (same config for builders for all ct).
   * \param gmt_config Config for the construction targets.
   * \param placement_penalty_config Config for the block placement penalty
   *                                 (same config for all all lanes for all
   *                                 ct).
   */
  crepr::config::nests_config initialize(
      const pgconfig::spct_builder_config* builder_config,
      const pgconfig::gmt_config* spct_config,
      const ctv::config::temporal_penalty_config* placement_penalty_config);

  /**
   * \brief The vector of all targets to be built in the arena.
   */
  const pds::ct_vectorno& targetsno(void) const { return m_targetsno; }
  const pds::ct_vectorro& targetsro(void) const { return m_targetsro; }

  const support::tv::block_op_penalty_handler*
  placement_handler(const rtypes::type_uuid& target_id) const {
    return m_envd->ct_penalty_handler(pstv::block_op_src::ekCT_BLOCK_MANIP,
                                      target_id);
  }
  support::tv::block_op_penalty_handler*
  placement_handler(const rtypes::type_uuid& target_id) {
    return m_envd->ct_penalty_handler(pstv::block_op_src::ekCT_BLOCK_MANIP,
                                      target_id);
  }
  const pstv::env_dynamics* env_dynamics(void) const { return m_envd; }
  pstv::env_dynamics* env_dynamics(void) { return m_envd; }

  const base_spct_builder*
  builder(const rtypes::type_uuid& target_id) const {
    return builder_lookup(target_id);
  }
  base_spct_builder* builder(const rtypes::type_uuid& target_id) {
    return const_cast<const ct_manager*>(this)->builder_lookup(target_id);
  }
  const spc_gmt* target(const rtypes::type_uuid& id) const {
    return target_lookup(id);
  }
  spc_gmt* target(const rtypes::type_uuid& id) {
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

  /**
   * \brief Returns \c TRUE if all CTs are complete, and \c FALSE otherwise.
   */
  bool targets_complete(void) const;

 private:
  /**
   * \brief Determine if construction of the specified target is feasible.
   *
   * - Does it pass internal validation?
   * - Does it overlap with other targets?
   */
  bool construction_feasible(const spc_gmt* target) const;

  using builders_vectoro_type =
      std::vector<std::unique_ptr<gmt::base_spct_builder>>;

  base_spct_builder*
  builder_lookup(const rtypes::type_uuid& target_id) const;
  spc_gmt* target_lookup(const rtypes::type_uuid& id) const;

  /* clang-format off */
  carena::base_arena_map* m_arena_map;
  cpal::argos_sm_adaptor* m_sm;
  pstv::env_dynamics*     m_envd;
  pds::ct_vectoro         m_targetso{};
  pds::ct_vectorno        m_targetsno{};
  pds::ct_vectorro        m_targetsro{};
  builders_vectoro_type   m_builderso{};
  /* clang-format on */
};

NS_END(gmt, prism);

#endif /* INCLUDE_PRISM_GMT_CT_MANAGER_HPP_ */
