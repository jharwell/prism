/**
 * \file robot_arena_interactor.hpp
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

#ifndef INCLUDE_SILICON_SUPPORT_ROBOT_ARENA_INTERACTOR_HPP_
#define INCLUDE_SILICON_SUPPORT_ROBOT_ARENA_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/types/timestep.hpp"

#include "silicon/silicon.hpp"
#include "silicon/support/interactor_status.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * \class robot_arena_interactor
 * \ingroup support
 *
 * \brief Handle's a robot's interactions with the environment on each timestep.
 *
 * Including:
 *
 */
template <typename T>
class robot_arena_interactor final
    : public rer::client<robot_arena_interactor<T>> {
 public:
  robot_arena_interactor(void)
      : ER_CLIENT_INIT("silicon.support.robot_arena_interactor") {}

  /**
   * \brief Interactors should generally NOT be copy constructable/assignable,
   * but is needed to use these classes with boost::variant.
   *
   * \todo Supposedly in recent versions of boost you can use variants with
   * move-constructible-only types (which is what this class SHOULD be), but I
   * cannot get this to work (the default move constructor needs to be noexcept
   * I think, and is not being interpreted as such).
   */
  robot_arena_interactor(const robot_arena_interactor&) = default;
  robot_arena_interactor& operator=(const robot_arena_interactor&) = delete;

  /**
   * \brief The actual handling function for the interactions.
   *
   * \param controller The controller to handle interactions for.
   * \param t The current timestep.
   */
  interactor_status operator()(T& controller, const rtypes::timestep& t) {
    if (controller.is_carrying_block()) {
      return m_nest_drop_interactor(controller, t);
    } else { /* The foot-bot has no block item */
      return m_free_pickup_interactor(controller, t);
    }
  }

 private:
  /* clang-format off */
  /* clang-format on */
};

NS_END(support, silicon);

#endif /* INCLUDE_SILICON_SUPPORT_ROBOT_ARENA_INTERACTOR_HPP_ */
