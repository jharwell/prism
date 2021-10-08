/**
 * \file placement_intent.hpp
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
#include <string>

#include "rcppsw/er/stringizable.hpp"

#include "prism/prism.hpp"
#include "prism/gmt/repr/ct_coord.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class placement_intent
 *
 * \ingroup repr
 *
 * \brief Representation of robot/arena intent to place a block at a specified
 * location with the specified orientation.
 */
class placement_intent : public rer::client<placement_intent>,
                         public rer::stringizable {
 public:
  placement_intent(void) : ER_CLIENT_INIT("prism.repr.placement_intent") {}

  placement_intent(const pgrepr::ct_coord& site, const rmath::radians& z_rot)
      : ER_CLIENT_INIT("prism.repr.placement_intent"),
        m_site(site),
        m_z_rot(z_rot) {}

  std::string to_str(void) const override {
    return "site=" + rcppsw::to_string(m_site) + "," +
           "z_rot=" + rcppsw::to_string(m_z_rot);
  }
  const pgrepr::ct_coord& site(void) const { return m_site; }
  const rmath::radians& z_rot(void) const { return m_z_rot; }

 private:
  /* clang-format off */
  pgrepr::ct_coord m_site{};
  rmath::radians   m_z_rot{};
  /* clang-format on */
};

NS_END(repr, prism);

