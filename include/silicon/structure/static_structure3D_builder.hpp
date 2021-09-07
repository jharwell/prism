/**
 * \file static_structure3D_builder.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_STATIC_STRUCTURE3D_BUILDER_HPP_
#define INCLUDE_SILICON_STRUCTURE_STATIC_STRUCTURE3D_BUILDER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>
#include <memory>

#include <argos3/plugins/simulator/entities/box_entity.h>

#include "rcppsw/er/client.hpp"

#include "cosm/ds/block3D_vector.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/repr/block_variant.hpp"

#include "silicon/silicon.hpp"
#include "silicon/structure/base_structure3D_builder.hpp"
#include "silicon/structure/config/structure3D_builder_config.hpp"
#include "silicon/structure/repr/ct_coord.hpp"
#include "silicon/structure/static_build_status.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class static_structure3D_builder
 * \ingroup structure
 *
 * \brief Action class for building structures directly from the loop functions
 * for debugging/testing purposes.
 */
class static_structure3D_builder : public rer::client<static_structure3D_builder>,
                                   public base_structure3D_builder {
 public:
  static_structure3D_builder(const config::structure3D_builder_config* config,
                             structure3D* target,
                             cpal::argos_sm_adaptor* sm);

  static_structure3D_builder(const static_structure3D_builder&) = default;
  const static_structure3D_builder&
  operator=(const static_structure3D_builder&) = delete;

  /* parent class overrides */
  void reset(void) override;
  void update(const rtypes::timestep& t,
              const cds::block3D_vectorno& blocks) override;

  /**
   * \brief Determine if static building via loop functions is enabled.
   */
  bool build_enabled(void) const;

  /**
   * \brief Build the ENTIRE structure, according to XML configuration (so many
   * blocks per timestep, etc.), in the loop functions without involving robots
   * at all. This is meant as debugging tool only for
   * diagnostic use in the loop functions; if you try to call it without the
   * proper XML configuration, it will trigger an assert.
   *
   * The build will take available (not carried by robot, in cache, etc.) blocks
   * from the provided vector as needed in order to complete the structure, and
   * bomb out if it cannot find enough.
   */
  static_build_status build(const rtypes::timestep& t,
                            const cds::block3D_vectorno& blocks);

 private:
  /**
   * \brief Bookeeping structure for loop function driven automated building of
   * structures.
   */
  struct static_build_state {
    size_t n_cells{ 0 };
    size_t n_built_interval{ 0 };
  };

  /**
   * \brief As part of \ref build_all(), find a block of the specified type from
   * the vector of provided blocks to add to the structure.
   *
   * \param type The type of block to try to find an available block for.
   * \param blocks The set of all blocks in the arena.
   * \param start The starting index within \p blocks, in order to reduce the
   *              complexity of repeated linear scans.
   */
  crepr::base_block3D* build_block_find(crepr::block_type type,
                                        const cds::block3D_vectorno& blocks,
                                        size_t start) const;

  /**
   * \brief Place a single block as part of \ref build().
   *
   * \return \c TRUE if a block was found to be placed AND placed successfully,
   * and \c FALSE otherwise.
   */
  bool build_single(const cds::block3D_vectorno& blocks, size_t search_start);

  /* clang-format off */
  const config::structure3D_builder_config mc_config;

  static_build_state                       m_static_state{};
  /* clang-format on */
};

NS_END(structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_STATIC_STRUCTURE3D_BUILDER_HPP_ */
