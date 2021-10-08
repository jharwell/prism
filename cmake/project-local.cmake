################################################################################
# Configuration Options                                                        #
################################################################################
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(PRISM_WITH_ROBOT_RAB "NO" CACHE STRING "Enable robots to read/write over the RAB medium via sensors/actuators.")
set(PRISM_WITH_ROBOT_BATTERY "NO" CACHE STRING "Enable robots to use the battery.")
set(PRISM_WITH_ROBOT_LEDS "YES" CACHE STRING "Enable robots to use their LEDs.")
set(PRISM_WITH_ROBOT_CAMERA "YES" CACHE STRING "Enable robots to use their camera.")

define_property(CACHED_VARIABLE PROPERTY "PRISM_WITH_ROBOT_RAB"
  BRIEF_DOCS "Enable robots to use the RAB medium."
  FULL_DOCS "Default=NO.")
define_property(CACHED_VARIABLE PROPERTY "PRISM_WITH_ROBOT_BATTERY"
  BRIEF_DOCS "Enable robots to use the battery."
  FULL_DOCS "Default=YES.")
define_property(CACHED_VARIABLE PROPERTY "PRISM_WITH_ROBOT_LEDS"
  BRIEF_DOCS "Enable robots to use their LEDs."
  FULL_DOCS "Default=YES.")
define_property(CACHED_VARIABLE PROPERTY "PRISM_WITH_ROBOT_CAMERA"
  BRIEF_DOCS "Enable robots to use their camera."
  FULL_DOCS "Default=YES.")


set(prism_CHECK_LANGUAGE "CXX")

################################################################################
# External Projects                                                            #
################################################################################
# COSM
if("${COSM_BUILD_FOR}" MATCHES "ARGOS_FOOTBOT")
  find_package(cosm-argos-footbot)
else()
  message(FATAL_ERROR "PRISM only supports [ARGOS_FOOTBOT] ${")
endif()

################################################################################
# Components
################################################################################
if ("${COSM_BUILD_FOR}" MATCHES "ARGOS")
  string(CONCAT argos_regex
    "src/fsm|"
      "src/controller|"
      "src/events|"
      "src/gmt|"
      "src/lane_alloc|"
      "src/metrics|"
      "src/repr|"
      "src/support"
      )
  component_register_as_src(
    prism_argos_SRC
    prism
    "${prism_SRC}"
    argos
    "(${argos_regex})")

  if(COSM_WITH_VIS)
    component_register_as_src(
      prism_argos_vis_SRC
      prism
      "${prism_SRC}"
      argos_vis
      "qt")
  endif()

  # Root project (not used in find_package())
  if (NOT prism_FIND_COMPONENTS)
    set(prism_FIND_COMPONENTS
      argos
      argos_vis
      )
  endif()
endif()

requested_components_check(prism)

################################################################################
# Libraries                                                                    #
################################################################################
# Create the source for the SINGLE library to build by combining the
# source of the selected components
foreach(component ${prism_FIND_COMPONENTS})
  if(${prism_${component}_FOUND})
    list(APPEND prism_components_SRC ${prism_} ${prism_${component}_SRC})
  endif()
endforeach()

# Define the PRISM library
set(prism_LIBRARY prism-${COSM_HAL_TARGET})

add_library(
  ${prism_LIBRARY}
  SHARED
  ${prism_components_SRC}
  )

# alias so we plug into the LIBRA framework properly
add_library(prism ALIAS ${prism_LIBRARY})

execute_process(COMMAND git rev-list --count HEAD
  OUTPUT_VARIABLE PRISM_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE)
set(prism_LIBRARY_NAME prism-${COSM_HAL_TARGET}-v${PRISM_VERSION})
set_target_properties(${prism_LIBRARY}
  PROPERTIES OUTPUT_NAME
  ${prism_LIBRARY_NAME})

########################################
# Include directories
########################################
target_include_directories(
  ${prism_LIBRARY}
  PUBLIC
  $<BUILD_INTERFACE:${prism_DIR}/include>
  $<BUILD_INTERFACE:${cosm_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>
  )

########################################
# Link Libraries
########################################
target_link_libraries(${prism_LIBRARY}
  cosm-${COSM_HAL_TARGET}::cosm-${COSM_HAL_TARGET}
  rt
  )

if ("${COSM_BUILD_FOR}" MATCHES "ARGOS")
  target_link_libraries(${prism_LIBRARY}
    argos3core_simulator
    argos3plugin_simulator_dynamics2d
    argos3plugin_simulator_entities
    argos3plugin_simulator_qtopengl
    argos3plugin_simulator_media
    argos3plugin_simulator_genericrobot
    argos3plugin_simulator_footbot
    # argos3plugin_simulator_epuck
    # argos3srocs_simulator_pipuck
  )
endif()

# Force failures at build time rather than runtime
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")

########################################
# Compile Options/Definitions
########################################
if ("${COSM_BUILD_FOR}" MATCHES "ARGOS")
  if (PRISM_WITH_ROBOT_RAB)
    target_compile_definitions(${prism_LIBRARY} PUBLIC PRISM_WITH_ROBOT_RAB)
  endif()
  if (PRISM_WITH_ROBOT_BATTERY)
    target_compile_definitions(${prism_LIBRARY} PUBLIC PRISM_WITH_ROBOT_BATTERY)
  endif()
  if (PRISM_WITH_ROBOT_CAMERA)
    target_compile_definitions(${prism_LIBRARY} PUBLIC PRISM_WITH_ROBOT_CAMERA)
  endif()
  if (PRISM_WITH_ROBOT_LEDS)
    target_compile_definitions(${prism_LIBRARY} PUBLIC PRISM_WITH_ROBOT_LEDS)
  endif()
endif()

if ("${COSM_BUILD_FOR}" MATCHES "MSI")
  target_compile_options(${prism_LIBRARY} PUBLIC
    -Wno-missing-include-dirs
    -fno-new-inheriting-ctors)
endif()

################################################################################
# Installation                                                                 #
################################################################################
configure_exports_as(${prism_LIBRARY} ${CMAKE_INSTALL_PREFIX})

# Install prism
register_target_for_install(${prism_LIBRARY} ${CMAKE_INSTALL_PREFIX})
register_headers_for_install(include/prism ${CMAKE_INSTALL_PREFIX})

################################################################################
# Status                                                                       #
################################################################################
libra_config_summary()

set(CMAKE_MODULE_PATH
  "${CMAKE_MODULE_PATH};${cosm_INSTALL_PREFIX}/lib/cmake/cosm-${COSM_HAL_TARGET}")
include(cosm-config-summary)
cosm_config_summary()

message(STATUS "")
message(STATUS "")
message(STATUS "PRISM Configuration Summary:")
message(STATUS "")

if(${COSM_BUILD_FOR} MATCHES "ARGOS")
  message(STATUS "With robot RAB........................: PRISM_WITH_ROBOT_RAB=${PRISM_WITH_ROBOT_RAB}")
  message(STATUS "With robot battery....................: PRISM_WITH_ROBOT_BATTERY=${PRISM_WITH_ROBOT_BATTERY}")
  message(STATUS "With robot LEDs.......................: PRISM_WITH_ROBOT_LEDS=${PRISM_WITH_ROBOT_LEDS}")
  message(STATUS "With robot CAMERA.....................: PRISM_WITH_ROBOT_CAMERA=${PRISM_WITH_ROBOT_CAMERA}")
endif()
