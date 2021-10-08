/**
 * \file constructing_controller.hpp
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
#include <boost/optional.hpp>
#include <memory>
#include <string>
#include <typeindex>

#include "cosm/controller/block_carrying_controller.hpp"
#include "cosm/controller/irv_recipient_controller.hpp"
#include "cosm/controller/manip_event_recorder.hpp"
#include "cosm/fsm/block_transporter.hpp"
#include "cosm/pal/config/output_config.hpp"
#include "cosm/pal/controller/controllerQ3D.hpp"
#include "cosm/subsystem/config/actuation_subsystem2D_config.hpp"
#include "cosm/hal/subsystem/config/sensing_subsystemQ3D_config.hpp"
#include "cosm/subsystem/subsystem_fwd.hpp"

#include "prism/fsm/block_placer.hpp"
#include "prism/fsm/construction_transport_goal.hpp"
#include "prism/metrics/blocks/block_manip_events.hpp"
#include "prism/controller/perception/config/perception_config.hpp"
#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class unicell_entity2D;
} // namespace cosm::repr

namespace prism::controller::perception {
class builder_perception_subsystem;
} /* namespace prism::controller::perception */

NS_START(prism, controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class constructing_controller
 * \ingroup controller
 *
 * \brief The base controller constructing class that all PRISM controllers
 * derive from. It holds all functionality common to all controllers, as well
 * that some that is stubbed out here, but overridden in derived classes which
 * allows this class to be used as the robot controller handle when rendering QT
 * graphics overlays.
 */
class constructing_controller
    : public cpcontroller::controllerQ3D,
      public ccontroller::irv_recipient_controller,
      public ccontroller::block_carrying_controller,
      public fsm::block_placer,
      public cfsm::block_transporter<fsm::construction_transport_goal>,
      public rer::client<constructing_controller> {
 public:
  using block_manip_recorder_type = ccontroller::manip_event_recorder<
      metrics::blocks::block_manip_events::ekMAX_EVENTS>;

  constructing_controller(void) RCPPSW_COLD;
  ~constructing_controller(void) override RCPPSW_COLD;

  constructing_controller(const constructing_controller&) = delete;
  constructing_controller& operator=(const constructing_controller&) = delete;

  /* controllerQ3D overrides */
  void init(ticpp::Element& node) override RCPPSW_COLD;
  void reset(void) override RCPPSW_COLD;
  rtypes::type_uuid entity_id(void) const override final;

  /* rda_recipient_controller overrides */
  double applied_movement_throttle(void) const override final;
  void irv_init(const ctv::robot_dynamics_applicator* rda) override final;

  /* block carrying controller overrides */
  bool block_detect(void) const override;

  /* movement metrics */
  rtypes::spatial_dist
  ts_distance(const csmetrics::movement_category& category) const override;
  rmath::vector3d
  ts_velocity(const csmetrics::movement_category& category) const override;

  virtual const perception::builder_perception_subsystem* perception(void) const {
    return m_perception.get();
  }
  virtual perception::builder_perception_subsystem* perception(void) {
    return m_perception.get();
  }

  double los_dim(void) const;

  bool display_nearest_ct(void) const { return m_display_nearest_ct; }
  void display_nearest_ct(bool b) { m_display_nearest_ct = b; }

  /**
   * \brief If \c TRUE, the robot is currently at least most of the way in the
   * nest, as reported by the sensors.
   */
  bool in_nest(void) const;

  const block_manip_recorder_type* block_manip_recorder(void) const {
    return &m_block_manip;
  }
  block_manip_recorder_type* block_manip_recorder(void) { return &m_block_manip; }
  const class csubsystem::saa_subsystemQ3D* saa(void) const {
    return cpcontroller::controllerQ3D::saa();
  }

 protected:
  class csubsystem::saa_subsystemQ3D* saa(void) {
    return cpcontroller::controllerQ3D::saa();
  }

 private:
  void
  saa_init(const csubsystem::config::actuation_subsystem2D_config* actuation_p,
           const chsubsystem::config::sensing_subsystemQ3D_config* sensing_p);

  fs::path output_init(const cpconfig::output_config* outputp) override;

  void perception_init(const perception::config::perception_config* perceptionp);

  /* clang-format off */
  bool                                                      m_display_nearest_ct{false};
  block_manip_recorder_type                                 m_block_manip{};
  std::unique_ptr<perception::builder_perception_subsystem> m_perception;
  /* clang-format on */
};

NS_END(controller, prism);

