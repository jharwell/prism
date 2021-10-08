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
#include "cosm/hal/argos/subsystem/config/xml/saa_names.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/steer2D/config/force_calculator_config.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/tv/robot_dynamics_applicator.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"
#include "cosm/subsystem/actuation_subsystem2D.hpp"

#include "prism/controller/config/constructing_controller_repository.hpp"
#include "prism/controller/perception/perception_subsystem_factory.hpp"
#include "prism/fsm/construction_transport_goal.hpp"
#include "prism/repr/diagnostics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, controller);

/*******************************************************************************
 * Static Functions
 ******************************************************************************/
#if defined(COSM_PAL_TARGET_ARGOS)
static csubsystem::sensing_subsystemQ3D::sensor_map
sensing_init(constructing_controller* c,
    const chsubsystem::config::sensing_subsystemQ3D_config* sensing) {
  using saa_names = chargos::subsystem::config::xml::saa_names;

  auto proximity_handle = c->GetSensor<chargos::sensors::ir_sensor::impl_type>(
      saa_names::ir_sensor);
  auto light_handle = c->GetSensor<chargos::sensors::light_sensor::impl_type>(
      saa_names::light_sensor);
  auto env_handle = c->GetSensor<chal::sensors::env_sensor::impl_type>(
      saa_names:: ground_sensor);
  auto position_handle = c->GetSensor<chargos::sensors::position_sensor::impl_type>(
      saa_names::position_sensor);
  auto steering_handle = c->GetSensor<chargos::sensors::diff_drive_sensor::impl_type>(
      saa_names::diff_steering_saa);

#if defined(PRISM_WITH_ROBOT_RAB)
  auto rab_handle = c->GetSensor<chsensors::wifi_sensor::impl_type>(
      saa_names::rab_saa);
#else
  auto rab_handle = nullptr;
#endif /* PRISM_WITH_ROBOT_RAB */

#if defined(PRISM_WITH_ROBOT_BATTERY)
  auto battery_handle = c->GetSensor<chsensors::battery_sensor::impl_type>(
      saa_names::battery_sensor);
#else
  auto battery_handle = nullptr;
#endif /* PRISM_WITH_ROBOT_BATTERY */

#ifdef PRISM_WITH_ROBOT_CAMERA
  auto blobs_handle = c->GetSensor<chargos::sensors::colored_blob_camera_sensor::impl_type>(
      saa_names::camera_sensor);
#else
      auto blobs_handle = nullptr;
#endif

      auto proximity = chsensors::proximity_sensor(proximity_handle,
                                                   &sensing->proximity);
      auto light = chargos::sensors::light_sensor(light_handle);
      auto env = chal::sensors::env_sensor(env_handle, &sensing->env);
      auto position = chargos::sensors::position_sensor(position_handle);
      auto steering = chargos::sensors::diff_drive_sensor(steering_handle);
      auto odometry = chal::sensors::odometry_sensor(std::move(position),
                                                     std::move(steering));

      auto rabs = chargos::sensors::wifi_sensor(rab_handle);
      auto battery = chargos::sensors::battery_sensor(battery_handle);
      auto blobs = chargos::sensors::colored_blob_camera_sensor(blobs_handle);


      using subsystem = csubsystem::sensing_subsystemQ3D;
      subsystem::sensor_map sensors;
#if defined(PRISM_WITH_ROBOT_RAB)
      sensors.emplace(subsystem::map_entry_create(std::move(rabs)));
#endif

#if defined(PRISM_WITH_ROBOT_BATTERY)
      sensors.emplace(subsystem::map_entry_create(std::move(battery)));
#endif
      sensors.emplace(subsystem::map_entry_create(std::move(proximity)));
      sensors.emplace(subsystem::map_entry_create(std::move(blobs)));
      sensors.emplace(subsystem::map_entry_create(std::move(light)));
      sensors.emplace(subsystem::map_entry_create(std::move(env)));
      sensors.emplace(subsystem::map_entry_create(std::move(odometry)));

      return sensors;
} /* sensing_init() */

static csubsystem::actuation_subsystem2D::actuator_map
actuation_init(constructing_controller* c,
               const csubsystem::config::actuation_subsystem2D_config* actuation) {
  using saa_names = chargos::subsystem::config::xml::saa_names;

  auto diff_drive = ckin2D::governed_diff_drive(
      &actuation->diff_drive,
      chactuators::diff_drive_actuator(
          c->GetActuator<chactuators::diff_drive_actuator::impl_type>(
              saa_names::diff_steering_saa)));

#ifdef PRISM_WITH_ROBOT_LEDS
  auto diag_handle = c->GetActuator<chargos::actuators::led_actuator::impl_type>(saa_names::leds_saa);
#else
  auto diag_handle = nullptr;
#endif /* PRISM_WITH_ROBOT_LEDS */
  auto diag = chactuators::diagnostic_actuator(diag_handle,
                                               prepr::diagnostics::kColorMap);

#if defined(PRISM_WITH_ROBOT_RAB)
  auto rab_handle = c->GetActuator<chactuators::wifi_actuator::impl_type>(saa_names::rab_saa);
#else
  auto rab_handle = nullptr;
#endif /* PRISM_WITH_ROBOT_RAB */

  auto rab = chargos::actuators::wifi_actuator(rab_handle);

  using subsystem = csubsystem::actuation_subsystem2D;
  subsystem::actuator_map actuators;
  actuators.emplace(subsystem::map_entry_create(std::move(diff_drive)));
  actuators.emplace(subsystem::map_entry_create(std::move(diag)));

#if defined(PRISM_WITH_ROBOT_RAB)
  actuators.emplace(subsystem::map_entry_create(std::move(rab)));
#endif
  return actuators;
} /* actuation_init() */
#endif

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
  mdc_ts_update();

  config::constructing_controller_repository repo;
  repo.parse_all(node);

  ndc_uuid_push();

  if (!repo.validate_all()) {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }

  /* initialize RNG */
  const auto* rngp = repo.config_get<rmath::config::rng_config>();
  rng_init((nullptr == rngp) ? -1 : rngp->seed, cpal::kRobotType);

  /* initialize output */
  output_init(repo.config_get<cpconfig::output_config>());

  /* initialize sensing and actuation (SAA) subsystem */
  saa_init(repo.config_get<csubsystem::config::actuation_subsystem2D_config>(),
           repo.config_get<chsubsystem::config::sensing_subsystemQ3D_config>());

  /* initialize perception */
  perception_init(repo.config_get<perception::config::perception_config>());

  /* initialize supervisor */
  supervisor(std::make_unique<cfsm::supervisor_fsm>(saa()));

  ndc_uuid_pop();
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
    const chsubsystem::config::sensing_subsystemQ3D_config* sensing_p) {
  auto actuators = actuation_init(this, actuation_p);
  auto sensors = sensing_init(this, sensing_p);

  auto saa = std::make_unique<csubsystem::saa_subsystemQ3D>(
      std::move(sensors),
      std::move(actuators),
      &actuation_p->steering);
  cpcontroller::controllerQ3D::saa(std::move(saa));
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
  return saa()->sensing()->env()->detect(
      chsensors::env_sensor::kNestTarget);
} /* in_nest() */

bool constructing_controller::block_detect(void) const {
  return saa()->sensing()->env()->detect("block");
} /* block_detect() */

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
