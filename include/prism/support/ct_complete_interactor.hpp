/**
 * \file ct_complete_interactor.hpp
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

#ifndef INCLUDE_PRISM_SUPPORT_CT_COMPLETE_INTERACTOR_HPP_
#define INCLUDE_PRISM_SUPPORT_CT_COMPLETE_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/mpl/at.hpp>

#include "cosm/controller/operations/stop.hpp"
#include "cosm/fsm/supervisor_fsm.hpp"

#include "prism/gmt/ct_manager.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, support);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
template <typename TController, typename TControllerSpecMap>
class ct_complete_interactor
    : public rer::client<ct_complete_interactor<TController, TControllerSpecMap>> {
 public:
  using controller_spec =
      typename boost::mpl::at<TControllerSpecMap, TController>::type;
  using interactor_status_type = typename controller_spec::interactor_status_type;

 public:
  explicit ct_complete_interactor(pgmt::ct_manager* const ct_manager)
      : ER_CLIENT_INIT("prism.support.ct_complete_interactor"),
        m_ct_manager(ct_manager) {}

  ct_complete_interactor(ct_complete_interactor&&) = default;

  /* Not copy-constructible/assignable by default. */
  ct_complete_interactor(const ct_complete_interactor&) = delete;
  ct_complete_interactor&
  operator=(const ct_complete_interactor&) = delete;

  /**
   * \brief The actual handling function for interactions.
   *
   * \param controller The controller to handle interactions for.
   * \param t The current timestep.
   */
  interactor_status_type operator()(TController& controller) {
    /*
     * Right now this only works if ALL ct are complete; this will need to
     * be refined later in the multi-target case if robots make any kind of
     * memory-based decisions on what structure to work on.
     */
    if (m_ct_manager->targets_complete() &&
        !controller.supervisor()->state_is_stopped()) {
      /* stop the controller */
      ccops::stop stop_op;
      stop_op.visit(controller);

      /* flush penalties */
      m_ct_manager->env_dynamics()->unregister_controller(controller);
      return interactor_status_type::ekROBOT_STOPPED;
    }
    return interactor_status_type::ekNO_EVENT;
  }

 private:
  /* clang-format off */
  pgmt::ct_manager* m_ct_manager;
  /* clang-format on */
};

NS_END(support, prism);

#endif /* INCLUDE_PRISM_SUPPORT_CT_COMPLETE_INTERACTOR_HPP_ */
