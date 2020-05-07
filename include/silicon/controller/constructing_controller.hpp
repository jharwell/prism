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
#include <boost/optional.hpp>

#include "cosm/controller/block_carrying_controller.hpp"
#include "cosm/controller/irv_recipient_controller.hpp"
#include "cosm/controller/manip_event_recorder.hpp"
#include "cosm/metrics/config/output_config.hpp"
#include "cosm/pal/argos_controllerQ3D_adaptor.hpp"
#include "cosm/robots/footbot/footbot_subsystem_fwd.hpp"
#include "cosm/subsystem/config/actuation_subsystem2D_config.hpp"
#include "cosm/subsystem/config/sensing_subsystemQ3D_config.hpp"
#include "cosm/controller/config/perception/perception_config.hpp"
#include "cosm/fsm/block_transporter.hpp"

#include "silicon/metrics/blocks/block_manip_events.hpp"
#include "silicon/silicon.hpp"
#include "silicon/fsm/construction_transport_goal.hpp"
#include "silicon/fsm/block_placer.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class unicell_entity2D;
} // namespace cosm::repr

namespace silicon::controller::perception {
class builder_perception_subsystem;
} /* namespace silicon::controller::perception */

NS_START(silicon, controller);

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
                                public fsm::block_placer,
                                public cfsm::block_transporter<fsm::construction_transport_goal>,

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

  virtual const perception::builder_perception_subsystem* perception(void) const {
    return m_perception.get();
  }
  virtual perception::builder_perception_subsystem* perception(void) {
    return m_perception.get();
  }

  double los_dim(void) const;

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
  class crfootbot::footbot_saa_subsystem* saa(void) RCSW_PURE;
  const class crfootbot::footbot_saa_subsystem* saa(void) const RCSW_PURE;

 private:
  void saa_init(
      const csubsystem::config::actuation_subsystem2D_config* actuation_p,
      const csubsystem::config::sensing_subsystemQ3D_config* sensing_p);
  void output_init(const cmconfig::output_config* outputp);

  void perception_init(
      const ccontconfig::perception::perception_config* perceptionp);

  /* clang-format off */
  block_manip_recorder_type                                 m_block_manip{};
  std::unique_ptr<perception::builder_perception_subsystem> m_perception;
  /* clang-format on */
};

NS_END(controller, silicon);

#endif /* INCLUDE_SILICON_CONTROLLER_CONSTRUCTING_CONTROLLER_HPP_ */
