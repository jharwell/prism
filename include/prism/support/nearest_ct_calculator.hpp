/**
 * \file nearest_ct_calculator.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include <boost/optional.hpp>

#include "rcppsw/math/vector3.hpp"

#include "prism/ds/ct_vector.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace prism::controller {
class constructing_controller;
} /* namespace prism::controller */

NS_START(prism, support);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class nearest_ct_calculator
 * \ingroup support
 *
 * \brief Calculates the nearest construction target to a given robot, along
 * with what the closest cell to the robot's current location.
 */
class nearest_ct_calculator : public rer::client<nearest_ct_calculator> {
 public:
  explicit nearest_ct_calculator(const pds::ct_vectorno& targets) :
      ER_CLIENT_INIT("prism.support.nearest_ct_calculator"),
      mc_targets(targets) {}

  /* Not move/copy constructable/assignable by default */
  nearest_ct_calculator(const nearest_ct_calculator&) = delete;
  nearest_ct_calculator& operator=(const nearest_ct_calculator&) = delete;
  nearest_ct_calculator(nearest_ct_calculator&&) = delete;
  nearest_ct_calculator& operator=(nearest_ct_calculator&&) = delete;

  gmt::spc_gmt* nearest_ct(const controller::constructing_controller* c) const;
  boost::optional<rmath::vector3z> nearest_ct_cell(
      const controller::constructing_controller* c,
      const gmt::spc_gmt* ct) const;

 private:
  /* clang-format off */
  const pds::ct_vectorno mc_targets;
  /* clang-format on */
};

NS_END(support, prism);

