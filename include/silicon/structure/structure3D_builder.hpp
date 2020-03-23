/**
 * \file structure3D_builder.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_STRUCTURE3D_BUILDER_HPP_
#define INCLUDE_SILICON_STRUCTURE_STRUCTURE3D_BUILDER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <boost/optional.hpp>

#include "rcppsw/er/client.hpp"

#include "cosm/repr/base_block3D.hpp"
#include "cosm/ds/block3D_vector.hpp"
#include "cosm/repr/block3D_variant.hpp"

#include "silicon/silicon.hpp"
#include "silicon/structure/config/structure3D_builder_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::pal {
class argos_sm_adaptor;
} /* namespace cosm::pal */

NS_START(silicon, structure);

class structure3D;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class structure3D_builder
 * \ingroup structure
 *
 * \brief Action class taking blocks placed by robots in the simulation and (1)
 * updating the \ref structure3D with the drop, and (2) updating ARGoS to
 * add the new embodied entity so that robots can interact with them. Up until
 * placement on the in-progress structure, 3D blocks are rendered/treated as 2D
 * for simplicity.
 */
class structure3D_builder : public rer::client<structure3D_builder> {
 public:
  /**
   * \brief XML key for specifying that the structure should be built entirely
   * by the loop functions.
   */
  static constexpr char kBuildSrcLoop[] = "loop";

  /**
   * \brief XML key for specifying that the structure should be built by robots.
   */
  static constexpr char kBuildSrcRobot[] = "robot";

  structure3D_builder(const config::structure3D_builder_config* config,
                      structure3D* target,
                      cpal::argos_sm_adaptor* sm);

  /* Not copy constructable/assignable by default */
  structure3D_builder(const structure3D_builder&) = delete;
  const structure3D_builder& operator=(const structure3D_builder&) = delete;

  bool block_placement_valid(const crepr::block3D_variant& block,
                             const rmath::vector3u& loc,
                             const rmath::radians& z_rotation) const;
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
   *
   * \return \c TRUE if all blocks placed this timestep were placed
   * successfully, \c FALSE otherwise.
   */
  bool build_static(const cds::block3D_vectorno& blocks,
                    const rtypes::timestep& t);

  /**
   * \brief Place the specified block onto the structure, update \ref
   * structure3D, and add physical embodied entity in simulation representing
   * the placed block.
   *
   * If the placement of the block fails validation checks, then no action is
   * performed.
   */
  bool place_block(const crepr::block3D_variant& block,
                   const rmath::vector3u& loc,
                   const rmath::radians& z_rotation);

 private:
  /**
   * \brief Bookeeping structure for loop function driven automated building of
   * structures.
   */
  struct static_build_state {
    size_t           interval_count{0};
    size_t           i{0};
    size_t           j{0};
    size_t           k{0};
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
  boost::optional<crepr::block3D_variant> build_block_find(crepr::block_type type,
                                                           const cds::block3D_vectorno& blocks,
                                                           size_t start) const;



  /* clang-format off */
  const config::structure3D_builder_config mc_config;

  static_build_state                       m_static_state{};
  structure3D*                             m_target;
  cpal::argos_sm_adaptor *                 m_sm;
  /* clang-format on */
};

NS_END(structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_STRUCTURE3D_BUILDER_HPP_ */
