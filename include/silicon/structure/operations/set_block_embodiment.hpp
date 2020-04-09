/**
 * \file set_block_embodiment.hpp
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

#ifndef INCLUDE_SILICON_STRUCTURE_OPERATIONS_SET_BLOCK_EMBODIMENT_HPP_
#define INCLUDE_SILICON_STRUCTURE_OPERATIONS_SET_BLOCK_EMBODIMENT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant/static_visitor.hpp>

#include "cosm/repr/embodied_block.hpp"
#include "cosm/repr/cube_block3D.hpp"
#include "cosm/repr/ramp_block3D.hpp"

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
struct set_block_embodiment_impl : public boost::static_visitor<void>,
                                   public rer::client<set_block_embodiment_impl> {
  set_block_embodiment_impl(void)
      : ER_CLIENT_INIT("silicon.structure.builder.block_embodiment") {}

  void operator()(crepr::cube_block3D* block,
                  const crepr::embodied_cube_block& embodiment) const {
    block->embodiment(boost::make_optional(embodiment));
  }
  void operator()(crepr::ramp_block3D* block,
                  const crepr::embodied_ramp_block& embodiment) const {
    block->embodiment(boost::make_optional(embodiment));
  }
  /*
   * Any other combinations of (block type, embodiment type) are bad, and we
   * should crash and burn if we ever hit them.
   */
  template <typename T, typename U>
  void operator()(T* , const U&) const noexcept {
  ER_FATAL_SENTINEL("Invalid (block type, embodiment) pair: (%s,%s)",
                    std::type_index(typeid(T)).name(),
                    std::type_index(typeid(U)).name());
  }
};

NS_END(detail);

/**
 * \struct set_block_embodiment
 * \ingroup structure
 *
 * \brief Functor to set the embodiment for blocks of any type in \ref
 * crepr::block3D_variant to the corresponding embodiment type in \ref
 * crepr::embodied_block_variant.
 */
struct set_block_embodiment : public boost::static_visitor<void> {
  void operator()(crepr::ramp_block3D* block,
                  const crepr::embodied_block_variant& embodiment) const {
    boost::apply_visitor(std::bind(detail::set_block_embodiment_impl(),
                                   block,
                                   std::placeholders::_1),
                         embodiment);
  }
  void operator()(crepr::cube_block3D* block,
                  const crepr::embodied_block_variant& embodiment) const {
    boost::apply_visitor(std::bind(detail::set_block_embodiment_impl(),
                                   block,
                                   std::placeholders::_1),
                         embodiment);
  }
};

NS_END(operations, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_OPERATIONS_SET_BLOCK_EMBODIMENT_HPP_ */
