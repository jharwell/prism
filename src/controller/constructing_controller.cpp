/**
 * \file constructing_controller.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "silicon/controller/constructing_controller.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <filesystem>
#include <fstream>

#include "rcppsw/math/config/rng_config.hpp"
#include "rcppsw/math/rngm.hpp"

#include "cosm/metrics/config/output_config.hpp"
#include "cosm/repr/base_block2D.hpp"
#include "cosm/robots/footbot/footbot_saa_subsystemQ3D.hpp"
#include "cosm/steer2D/config/force_calculator_config.hpp"
#include "cosm/subsystem/config/actuation_subsystem2D_config.hpp"
#include "cosm/subsystem/config/sensing_subsystem2D_config.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/tv/robot_dynamics_applicator.hpp"
#include "cosm/fsm/supervisor_fsm.hpp"

#include "silicon/controller/config/constructing_controller_repository.hpp"
#include "cosm/robots/footbot/config/saa_xml_names.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, controller);
namespace fs = std::filesystem;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
constructing_controller::constructing_controller(void)
    : ER_CLIENT_INIT("silicon.controller") {}

constructing_controller::~constructing_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool constructing_controller::in_nest(void) const {
  return saa()->sensing()->ground()->detect(
      chal::sensors::ground_sensor::kNestTarget);
} /* in_nest() */

bool constructing_controller::block_detected(void) const {
  return saa()->sensing()->ground()->detect("block");
} /* block_detected() */

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
  auto rngp = repo.config_get<rmath::config::rng_config>();
  base_controllerQ3D::rng_init((nullptr == rngp) ? -1 : rngp->seed,
                              kARGoSRobotType);

  /* initialize output */
  auto* outputp = repo.config_get<cmconfig::output_config>();
  base_controllerQ3D::output_init(outputp->output_root, outputp->output_dir);

  /* initialize sensing and actuation (SAA) subsystem */
  saa_init(repo.config_get<csubsystem::config::actuation_subsystem2D_config>(),
           repo.config_get<csubsystem::config::sensing_subsystem2D_config>());

  /* initialize supervisor */
  supervisor(std::make_unique<cfsm::supervisor_fsm>(saa()));
  ndc_pop();
} /* init() */

void constructing_controller::reset(void) { block_carrying_controller::reset(); }

void constructing_controller::output_init(const cmconfig::output_config* outputp) {
  std::string dir =
      base_controllerQ3D::output_init(outputp->output_root, outputp->output_dir);

#if (LIBRA_ER == LIBRA_ER_ALL)
  /*
   * Each file appender is attached to a root category in the SILICON
   * namespace. If you give different file appenders the same file, then the
   * lines within it are not always ordered, which is not overly helpful for
   * debugging.
   */
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("cosm.ta"), dir + "/ta.log");

  ER_LOGFILE_SET(log4cxx::Logger::getLogger("silicon.controller"),
                 dir + "/controller.log");
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("silicon.structure"),
                 dir + "/structure.log");
#endif
} /* output_init() */

void constructing_controller::saa_init(
    const csubsystem::config::actuation_subsystem2D_config* actuation_p,
    const csubsystem::config::sensing_subsystem2D_config* sensing_p) {
  auto saa_names = crfootbot::config::saa_xml_names();

  auto position = chal::sensors::position_sensor(
      GetSensor<argos::CCI_PositioningSensor>(saa_names.position_sensor));
  auto proximity = chal::sensors::proximity_sensor(
      GetSensor<argos::CCI_FootBotProximitySensor>(saa_names.prox_sensor),
      &sensing_p->proximity);
  auto blobs = chal::sensors::colored_blob_camera_sensor(
      GetSensor<argos::CCI_ColoredBlobOmnidirectionalCameraSensor>(
          saa_names.camera_sensor));
  auto light = chal::sensors::light_sensor(
      GetSensor<argos::CCI_FootBotLightSensor>(saa_names.light_sensor));
  auto ground = chal::sensors::ground_sensor(
      GetSensor<argos::CCI_FootBotMotorGroundSensor>(saa_names.ground_sensor),
      &sensing_p->ground);

  auto diff_drives = chal::sensors::diff_drive_sensor(
      GetSensor<argos::CCI_DifferentialSteeringSensor>(
          saa_names.diff_steering_saa));

  auto sensors = csubsystem::sensing_subsystemQ3D::sensor_map{
      csubsystem::sensing_subsystemQ3D::map_entry_create(proximity),
      csubsystem::sensing_subsystemQ3D::map_entry_create(blobs),
      csubsystem::sensing_subsystemQ3D::map_entry_create(light),
      csubsystem::sensing_subsystemQ3D::map_entry_create(ground),
      csubsystem::sensing_subsystemQ3D::map_entry_create(diff_drives)};

  auto diff_drivea = ckin2D::governed_diff_drive(
      &actuation_p->diff_drive,
      chal::actuators::diff_drive_actuator(
          GetActuator<argos::CCI_DifferentialSteeringActuator>(
              saa_names.diff_steering_saa)),
      ckin2D::governed_diff_drive::drive_type::kFSMDrive);

#ifdef COSM_WITH_ARGOS_ROBOT_LEDS
  auto leds = chal::actuators::led_actuator(
      GetActuator<argos::CCI_LEDsActuator>(saa_names.leds_saa));
#else
  auto leds = chal::actuators::led_actuator(nullptr);
#endif /* COSM_WITH_ARGOS_ROBOT_LEDS */

  auto actuators = csubsystem::actuation_subsystem2D::actuator_map{
    /*
     * We put the governed differential drive in the actuator map twice because
     * some of the reusable components use the base class differential drive
     * instead of the governed version (no robust way to inform that we want to
     * use the governed version).
     */
      csubsystem::actuation_subsystem2D::map_entry_create(diff_drivea),
      csubsystem::actuation_subsystem2D::map_entry_create(leds)};

  base_controllerQ3D::saa(std::make_unique<crfootbot::footbot_saa_subsystemQ3D>(
      position, sensors, actuators, &actuation_p->steering));
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

class crfootbot::footbot_saa_subsystemQ3D* constructing_controller::saa(void) {
  return static_cast<crfootbot::footbot_saa_subsystemQ3D*>(
      base_controllerQ3D::saa());
}
const class crfootbot::footbot_saa_subsystemQ3D* constructing_controller::saa(
    void) const {
  return static_cast<const crfootbot::footbot_saa_subsystemQ3D*>(
      base_controllerQ3D::saa());
}
NS_END(controller, silicon);
