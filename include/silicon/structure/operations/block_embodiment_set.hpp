/**
 * \file block_embodiment_set.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_OPERATIONS_BLOCK_EMBODIMENT_SET_HPP_
#define INCLUDE_SILICON_STRUCTURE_OPERATIONS_BLOCK_EMBODIMENT_SET_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant/static_visitor.hpp>
#include <memory>
#include <utility>

#include "cosm/pal/block_embodiment_variant.hpp"
#include "cosm/pal/embodied_cube_block.hpp"
#include "cosm/pal/embodied_ramp_block.hpp"

#include "silicon/silicon.hpp"
#include "silicon/structure/structure3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::foraging::ds {
class arena_map;
} /* namespace cosm::foraging::ds */

NS_START(silicon, structure, operations, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
struct block_embodiment_set_impl : public rer::client<block_embodiment_set_impl>,
                                   private boost::static_visitor<void> {
  block_embodiment_set_impl(void)
      : ER_CLIENT_INIT("silicon.structure.builder.block_embodiment_set") {}
  virtual ~block_embodiment_set_impl(void) = default;

  template <typename T>
  void operator()(T* block,
                  typename T::embodiment_type embodiment) const {
    block->embodiment(std::move(embodiment));
  }
  /*
   * Any combinations of (block type, embodiment type) not as defined in each
   * embodied block class are bad, and we should crash and burn if we ever hit
   * them.
   */
  template <typename T, typename U>
  void operator()(T* , U) const noexcept {
    static_assert("Invalid (block type, embodiment) pair");
  }
};

NS_END(detail);

/**
 * \struct block_embodiment_set
 * \ingroup structure
 *
 * \brief Functor to set the embodiment for blocks of any type in \ref
 * crepr::block3D_variant to the corresponding embodiment type in \ref
 * crepr::embodied_block_variant.
 */
struct block_embodiment_set : public boost::static_visitor<void> {
  explicit block_embodiment_set(cpal::block_embodiment_variant&& e)
      : embodiment(std::move(e)) {}

  void operator()(cpal::embodied_ramp_block* block) {
    boost::apply_visitor(std::bind(detail::block_embodiment_set_impl(),
                                   block,
                                   std::placeholders::_1),
                         std::move(embodiment));
  }
  void operator()(cpal::embodied_cube_block* block) {
    boost::apply_visitor(std::bind(detail::block_embodiment_set_impl(),
                                   block,
                                   std::placeholders::_1),
                         std::move(embodiment));
  }
  cpal::block_embodiment_variant embodiment;
};

NS_END(operations, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_OPERATIONS_BLOCK_EMBODIMENT_SET_HPP_ */
