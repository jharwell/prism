/**
 * \file constructing_controller.hpp
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

#ifndef INCLUDE_SILICON_CONTROLLER_CONSTRUCTING_CONTROLLER_HPP_
#define INCLUDE_SILICON_CONTROLLER_CONSTRUCTING_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>
#include <typeindex>

#include "cosm/controller/block_carrying_controller.hpp"
#include "cosm/controller/irv_recipient_controller.hpp"
#include "cosm/controller/manip_event_recorder.hpp"
#include "cosm/metrics/config/output_config.hpp"
#include "cosm/pal/argos_controllerQ3D_adaptor.hpp"
#include "cosm/repr/base_block2D.hpp"
#include "cosm/robots/footbot/footbot_subsystem_fwd.hpp"

#include "silicon/metrics/blocks/block_manip_events.hpp"
#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem::config {
struct actuation_subsystem2D_config;
struct sensing_subsystem2D_config;
} // namespace cosm::subsystem::config

namespace cosm::repr {
class unicell_entity2D;
} // namespace cosm::repr

namespace rcppsw::math::config {
struct rng_config;
} // namespace rcppsw::math::config

NS_START(silicon);

namespace repr {
class line_of_sight;
} // namespace repr

NS_START(controller);
class base_perception_subsystem;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class constructing_controller
 * \ingroup controller
 *
 * \brief The base controller constructing class that all SILICON controllers
 * derive from. It holds all functionality common to all controllers, as well
 * that some that is stubbed out here, but overridden in derived classes which
 * allows this class to be used as the robot controller handle when rendering QT
 * graphics overlays.
 */
class constructing_controller : public cpal::argos_controllerQ3D_adaptor,
                                public ccontroller::irv_recipient_controller,
                                public ccontroller::block_carrying_controller,
                                public rer::client<constructing_controller> {
 public:
  using block_manip_recorder_type = ccontroller::manip_event_recorder<
      metrics::blocks::block_manip_events::ekMAX_EVENTS>;

  constructing_controller(void) RCSW_COLD;
  ~constructing_controller(void) override RCSW_COLD;

  constructing_controller(const constructing_controller&) = delete;
  constructing_controller& operator=(const constructing_controller&) = delete;

  /* constructing_controllerQ3D overrides */
  void init(ticpp::Element& node) override RCSW_COLD;
  void reset(void) override RCSW_COLD;
  rtypes::type_uuid entity_id(void) const override final;

  /* rda_recipient_controller overrides */
  double applied_movement_throttle(void) const override final;
  void irv_init(const ctv::robot_dynamics_applicator* rda) override final;

  /* block carrying controller overrides */
  bool block_detected(void) const override;

  /**
   * \brief By default controllers have no perception subsystem, and are
   * basically blind centipedes.
   */
  virtual const base_perception_subsystem* perception(void) const {
    return nullptr;
  }

  /**
   * \brief By default controllers have no perception subsystem, and are
   * basically blind centipedes.
   */
  virtual base_perception_subsystem* perception(void) { return nullptr; }

  /**
   * \brief If \c TRUE, the robot is currently at least most of the way in the
   * nest, as reported by the sensors.
   */
  bool in_nest(void) const;

  const block_manip_recorder_type* block_manip_recorder(void) const {
    return &m_block_manip;
  }
  block_manip_recorder_type* block_manip_recorder(void) {
    return &m_block_manip;
  }

 protected:
  class crfootbot::footbot_saa_subsystemQ3D* saa(void) RCSW_PURE;
  const class crfootbot::footbot_saa_subsystemQ3D* saa(void) const RCSW_PURE;

 private:
  void saa_init(
      const csubsystem::config::actuation_subsystem2D_config* actuation_p,
      const csubsystem::config::sensing_subsystem2D_config* sensing_p);
  void output_init(const cmconfig::output_config* outputp);

  /* clang-format off */
  block_manip_recorder_type m_block_manip{};
  /* clang-format on */
};

NS_END(controller, silicon);

#endif /* INCLUDE_SILICON_CONTROLLER_CONSTRUCTING_CONTROLLER_HPP_ */
