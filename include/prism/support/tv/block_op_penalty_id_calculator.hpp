/**
 * \file block_op_penalty_id_calculator.hpp
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
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/foraging/tv/penalty_id_calculator.hpp"
#include "cosm/arena/base_arena_map.hpp"

#include "prism/prism.hpp"
#include "prism/support/tv/block_op_src.hpp"
#include "prism/controller/controller_fwd.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, support, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_op_penalty_id_calculator
 * \ingroup support tv
 *
 * \brief Extents \ref ctv::penalty_id_calculator with additional
 * PRISM-specific ways to calculate the ID that should be associated with a
 * particular temporal penalty.
 */
class block_op_penalty_id_calculator : public rer::client<block_op_penalty_id_calculator>,
                                       public rpdecorator::decorator<cforaging::tv::penalty_id_calculator> {
 public:
  explicit block_op_penalty_id_calculator(const carena::base_arena_map* map);

  /* Not copy constructable/assignable by default */
  block_op_penalty_id_calculator(const block_op_penalty_id_calculator&) = delete;
  const block_op_penalty_id_calculator& operator=(const block_op_penalty_id_calculator&) = delete;

  rtypes::type_uuid operator()(const controller::constructing_controller& controller,
                               const block_op_src& src) const;

 private:
  /* clang-formatoff */
  const carena::base_arena_map* mc_map;
  /* clang-format on */
};

NS_END(tv, support, prism);

