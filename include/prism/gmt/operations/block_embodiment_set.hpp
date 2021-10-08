/**
 * \file block_embodiment_set.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <variant>
#include <memory>
#include <utility>

#include "cosm/argos/block_embodiment_variant.hpp"
#include "cosm/argos/embodied_cube_block.hpp"
#include "cosm/argos/embodied_ramp_block.hpp"

#include "prism/prism.hpp"
#include "prism/gmt/spc_gmt.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::foraging::ds {
class arena_map;
} /* namespace cosm::foraging::ds */

NS_START(prism, gmt, operations, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
struct block_embodiment_set_impl : public rer::client<block_embodiment_set_impl> {
  block_embodiment_set_impl(void)
      : ER_CLIENT_INIT("prism.gmt.builder.block_embodiment_set") {}
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
 * \ingroup gmt
 *
 * \brief Functor to set the embodiment for blocks of any type in \ref
 * crepr::block3D_variant to the corresponding embodiment type in \ref
 * crepr::embodied_block_variant.
 */
struct block_embodiment_set {
  explicit block_embodiment_set(cargos::block_embodiment_variant&& e)
      : embodiment(std::move(e)) {}

  void operator()(cargos::embodied_ramp_block* block) {
    std::visit(std::bind(detail::block_embodiment_set_impl(),
                         block,
                         std::placeholders::_1),
               std::move(embodiment));
  }
  void operator()(cargos::embodied_cube_block* block) {
    std::visit(std::bind(detail::block_embodiment_set_impl(),
                         block,
                         std::placeholders::_1),
               std::move(embodiment));
  }
  cargos::block_embodiment_variant embodiment;
};

NS_END(operations, gmt, prism);
