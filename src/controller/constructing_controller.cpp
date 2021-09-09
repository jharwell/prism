/**
 * \file constructing_controller.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/controller/constructing_controller.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <fstream>

#include "rcppsw/math/config/rng_config.hpp"
#include "rcppsw/math/rngm.hpp"

#include "cosm/fsm/supervisor_fsm.hpp"
#include "cosm/hal/subsystem/config/saa_xml_names.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/steer2D/config/force_calculator_config.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/tv/robot_dynamics_applicator.hpp"

#include "prism/controller/config/constructing_controller_repository.hpp"
#include "prism/controller/perception/perception_subsystem_factory.hpp"
#include "prism/fsm/construction_transport_goal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
constructing_controller::constructing_controller(void)
    : ER_CLIENT_INIT("prism.controller"), m_perception(nullptr) {}

constructing_controller::~constructing_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void constructing_controller::init(ticpp::Element& node) {
  /* verify environment variables set up for logging */
  ER_ENV_VERIFY();

  config::constructing_controller_repository repo;
  repo.parse_all(node);

  ndc_push();
  if (!repo.validate_all()) {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }

  /* initialize RNG */
  const auto* rngp = repo.config_get<rmath::config::rng_config>();
  rng_init((nullptr == rngp) ? -1 : rngp->seed, cpal::kARGoSRobotType);

  /* initialize output */
  output_init(repo.config_get<cpconfig::output_config>());

  /* initialize sensing and actuation (SAA) subsystem */
  saa_init(repo.config_get<csubsystem::config::actuation_subsystem2D_config>(),
           repo.config_get<csubsystem::config::sensing_subsystemQ3D_config>());

  /* initialize perception */
  perception_init(repo.config_get<perception::config::perception_config>());

  /* initialize supervisor */
  supervisor(std::make_unique<cfsm::supervisor_fsm>(saa()));

  ndc_pop();
} /* init() */

double constructing_controller::los_dim(void) const {
  return perception()->los_dim();
} /* los_dim() */

void constructing_controller::reset(void) { block_carrying_controller::reset(); }

void constructing_controller::perception_init(
    const perception::config::perception_config* perceptionp) {
  auto factory = perception::perception_subsystem_factory();
  m_perception = factory.create(perceptionp->type,
                                &perceptionp->rlos,
                                saa()->sensing());
} /* perception_init() */

fs::path constructing_controller::output_init(
    const cpconfig::output_config* outputp) {
  auto path = base_controllerQ3D::output_init(outputp);

#if (LIBRA_ER == LIBRA_ER_ALL)
  /*
   * Each file appender is attached to a root category in the PRISM
   * namespace. If you give different file appenders the same file, then the
   * lines within it are not always ordered, which is not overly helpful for
   * debugging.
   */
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("cosm.ta"), path / "ta.log");

  ER_LOGFILE_SET(log4cxx::Logger::getLogger("prism.controller"),
                 path / "controller.log");
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("prism.structure"),
                 path / "structure.log");
#endif
  return path;
} /* output_init() */

void constructing_controller::saa_init(
    const csubsystem::config::actuation_subsystem2D_config* actuation_p,
    const csubsystem::config::sensing_subsystemQ3D_config* sensing_p) {
  using saa_names = chsubsystem::config::saa_xml_names;

  auto position = chal::sensors::position_sensor(
      GetSensor<chal::sensors::position_sensor::impl_type>(
          saa_names::position_sensor));
  auto proximity = chal::sensors::proximity_sensor(
      GetSensor<chal::sensors::proximity_sensor::impl_type>(
          saa_names::prox_sensor),
      &sensing_p->proximity);

#ifdef PRISM_WITH_ROBOT_CAMERA
  auto blobs = chal::sensors::colored_blob_camera_sensor(
      GetSensor<chal::sensors::colored_blob_camera_sensor::impl_type>(
          saa_names::camera_sensor));
#else
  auto blobs = chal::sensors::colored_blob_camera_sensor(nullptr);
#endif /* PRISM_WITH_ROBOT_CAMERA */

  auto light = chal::sensors::light_sensor(
      GetSensor<chal::sensors::light_sensor::impl_type>(saa_names::light_sensor));
  auto ground = chal::sensors::ground_sensor(
      GetSensor<chal::sensors::ground_sensor::impl_type>(
          saa_names::ground_sensor),
      &sensing_p->ground);

  auto diff_drives = chal::sensors::diff_drive_sensor(
      GetSensor<chal::sensors::diff_drive_sensor::impl_type>(
          saa_names::diff_steering_saa));

  auto sensors = csubsystem::sensing_subsystemQ3D::sensor_map{
    csubsystem::sensing_subsystemQ3D::map_entry_create(proximity),
    csubsystem::sensing_subsystemQ3D::map_entry_create(position),
    csubsystem::sensing_subsystemQ3D::map_entry_create(blobs),
    csubsystem::sensing_subsystemQ3D::map_entry_create(light),
    csubsystem::sensing_subsystemQ3D::map_entry_create(ground),
    csubsystem::sensing_subsystemQ3D::map_entry_create(diff_drives)
  };

  auto diff_drivea = ckin2D::governed_diff_drive(
      &actuation_p->diff_drive,
      chal::actuators::diff_drive_actuator(
          GetActuator<chal::actuators::diff_drive_actuator::impl_type>(
              saa_names::diff_steering_saa)),
      ckin2D::governed_diff_drive::drive_type::ekFSM_DRIVE);

#ifdef PRISM_WITH_ROBOT_LEDS
  auto leds = chal::actuators::led_actuator(
      GetActuator<chal::actuators::led_actuator::impl_type>(saa_names::leds_saa));
#else
  auto leds = chal::actuators::led_actuator(nullptr);
#endif /* PRISM_WITH_ROBOT_LEDS */

  auto actuators = csubsystem::actuation_subsystem2D::actuator_map{
    /*
       * We put the governed differential drive in the actuator map twice
       * because some of the reusable components use the base class differential
       * drive instead of the governed version (no robust way to inform that we
       * want to use the governed version).
       */
    csubsystem::actuation_subsystem2D::map_entry_create(
        chal::actuators::diff_drive_actuator(
            GetActuator<chal::actuators::diff_drive_actuator::impl_type>(
                saa_names::diff_steering_saa))),
    csubsystem::actuation_subsystem2D::map_entry_create(diff_drivea),
    csubsystem::actuation_subsystem2D::map_entry_create(leds)
  };

  base_controllerQ3D::saa(std::make_unique<csubsystem::saa_subsystemQ3D>(
      sensors, actuators, &actuation_p->steering));
} /* saa_init() */

rtypes::type_uuid constructing_controller::entity_id(void) const {
  return rtypes::type_uuid(std::atoi(GetId().c_str() + 2));
} /* entity_id() */

double constructing_controller::applied_movement_throttle(void) const {
  return saa()->actuation()->governed_diff_drive()->applied_throttle();
} /* applied_movement_throttle() */

void constructing_controller::irv_init(const ctv::robot_dynamics_applicator* rda) {
  if (rda->motion_throttling_enabled()) {
    saa()->actuation()->governed_diff_drive()->tv_generator(
        rda->motion_throttler(entity_id()));
  }
} /* irv_init() */

bool constructing_controller::in_nest(void) const {
  return saa()->sensing()->ground()->detect(
      chal::sensors::ground_sensor::kNestTarget);
} /* in_nest() */

bool constructing_controller::block_detected(void) const {
  return saa()->sensing()->ground()->detect("block");
} /* block_detected() */

/*******************************************************************************
 * Movement Metrics
 ******************************************************************************/
rtypes::spatial_dist constructing_controller::ts_distance(
    const csmetrics::movement_category& category) const {
  if (csmetrics::movement_category::ekALL == category) {
    return ts_distance_impl();
  } else if (csmetrics::movement_category::ekHOMING == category) {
    if (fsm::construction_transport_goal::ekCONSTRUCTION_SITE ==
        block_transport_goal()) {
      return ts_distance_impl();
    }
  }
  return rtypes::spatial_dist(0);
} /* ts_distance() */

rmath::vector3d constructing_controller::ts_velocity(
    const csmetrics::movement_category& category) const {
  if (csmetrics::movement_category::ekALL == category) {
    return ts_velocity_impl();
  } else if (csmetrics::movement_category::ekHOMING == category) {
    if (fsm::construction_transport_goal::ekCONSTRUCTION_SITE ==
        block_transport_goal()) {
      return ts_velocity_impl();
    }
  }
  return {};
} /* ts_velocity() */

NS_END(controller, prism);
