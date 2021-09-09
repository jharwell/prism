/**
 * \file prism_pd_adaptor.hpp
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

#ifndef INCLUDE_PRISM_SUPPORT_TV_PRISM_PD_ADAPTOR_HPP_
#define INCLUDE_PRISM_SUPPORT_TV_PRISM_PD_ADAPTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/pal/tv/argos_pd_adaptor.hpp"
#include "cosm/pal/argos_controllerQ3D_adaptor.hpp"
#include "cosm/repr/base_block3D.hpp"

#include "prism/controller/controller_fwd.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::arena {
class base_arena_map;
} /* namespace cosm::arena */

NS_START(prism, support, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class prism_pd_adaptor
 * \ingroup support tv
 *
 * \brief Further adapts \ref ctv::argos_pd_adaptor to the PRISM project,
 * providing additional callbacks to maintain simulation consistency/fidelity
 * during population dynamics application.
 */
class prism_pd_adaptor : rer::client<prism_pd_adaptor>,
                           public cptv::argos_pd_adaptor<cpal::argos_controllerQ3D_adaptor>{
 public:
  prism_pd_adaptor(const ctv::config::population_dynamics_config* config,
                     cpal::argos_sm_adaptor* sm,
                     env_dynamics_type *envd,
                     carena::base_arena_map* map,
                     rmath::rng* rng);

  /* Not copy constructable/assignable by default */
  prism_pd_adaptor(const prism_pd_adaptor&) = delete;
  const prism_pd_adaptor& operator=(const prism_pd_adaptor&) = delete;

  /* ARGoS PD apdaptor overrides */
  void pre_kill_cleanup(cpal::argos_controllerQ3D_adaptor* controller) override;

 private:
  /* clang-format off */
  carena::base_arena_map* m_map;
  /* clang-format on */
};

NS_END(tv, support, prism);

#endif /* INCLUDE_PRISM_SUPPORT_TV_PRISM_PD_ADAPTOR_HPP_ */
