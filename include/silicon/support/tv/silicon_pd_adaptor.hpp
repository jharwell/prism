/**
 * \file silicon_pd_adaptor.hpp
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

#ifndef INCLUDE_SILICON_SUPPORT_TV_SILICON_PD_ADAPTOR_HPP_
#define INCLUDE_SILICON_SUPPORT_TV_SILICON_PD_ADAPTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/pal/tv/argos_pd_adaptor.hpp"
#include "cosm/pal/argos_controllerQ3D_adaptor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace silicon::controller {
class constructing_controller;
} /* namespace silicon::controller */

NS_START(silicon, support, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class silicon_pd_adaptor
 * \ingroup support tv
 *
 * \brief Further adapts \ref ctv::argos_pd_adaptor to the SILICON project,
 * providing additional callbacks to maintain simulation consistency/fidelity
 * during population dynamics application.
 */
class silicon_pd_adaptor : rer::client<silicon_pd_adaptor>,
                           public cptv::argos_pd_adaptor<cpal::argos_controllerQ3D_adaptor>{
 public:
  template <typename... Args>
  silicon_pd_adaptor(Args&&... args)
      : ER_CLIENT_INIT("silicon.support.tv.silicon_pd_adaptor"),
        argos_pd_adaptor<cpal::argos_controllerQ3D_adaptor>(std::forward<Args>(args)...) {}

  /* Not copy constructable/assignable by default */
  silicon_pd_adaptor(const silicon_pd_adaptor&) = delete;
  const silicon_pd_adaptor& operator=(const silicon_pd_adaptor&) = delete;

  /* ARGoS PD apdaptor overrides */
  void pre_kill_cleanup(cpal::argos_controllerQ3D_adaptor* controller) override;
};

NS_END(tv, support, silicon);

#endif /* INCLUDE_SILICON_SUPPORT_TV_SILICON_PD_ADAPTOR_HPP_ */
