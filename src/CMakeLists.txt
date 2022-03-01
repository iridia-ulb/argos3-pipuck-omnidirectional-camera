cmake_minimum_required(VERSION 2.8.12)
project(argos3-pipuck-omnidirectional-camera)

# Use new policies introduced up to this version
cmake_policy(SET CMP0042 NEW)
cmake_policy(SET CMP0071 NEW)
set(OpenGL_GL_PREFERENCE "LEGACY")

# Make sure we are under Unix
if(NOT UNIX)
  message(FATAL_ERROR "ARGoS compiles only under UNIX, sorry!")
endif(NOT UNIX)

set(CMAKE_MACOSX_RPATH 0)
set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR})

include(${CMAKE_SOURCE_DIR}/cmake/ARGoSBuildOptions.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/ARGoSBuildFlags.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/ARGoSBuildChecks.cmake)

#
# Headers
#
set(ARGOS3_PIPUCK_OMNIDIRECTIONAL_CAMERA_HEADERS_CONTROLINTERFACE
  plugins/robots/pi-puck/control_interface/ci_pipuck_omnidirectional_camera_sensor.h)
set(ARGOS3_PIPUCK_OMNIDIRECTIONAL_CAMERA_HEADERS_SIMULATOR
  plugins/robots/pi-puck/simulator/pipuck_omnidirectional_camera_default_sensor.h)
set(ARGOS3_PIPUCK_OMNIDIRECTIONAL_CAMERA_HEADERS_HARDWARE
  plugins/robots/pi-puck/hardware/pipuck_omnidirectional_camera_default_sensor.h)

#
# Sources
#
set(ARGOS3_PIPUCK_OMNIDIRECTIONAL_CAMERA_SOURCES
  ${ARGOS3_PIPUCK_OMNIDIRECTIONAL_CAMERA_HEADERS_CONTROLINTERFACE}
  plugins/robots/pi-puck/control_interface/ci_pipuck_omnidirectional_camera_sensor.cpp)
if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_PIPUCK_OMNIDIRECTIONAL_CAMERA_SOURCES
    ${ARGOS3_PIPUCK_OMNIDIRECTIONAL_CAMERA_SOURCES}
    ${ARGOS3_PIPUCK_OMNIDIRECTIONAL_CAMERA_HEADERS_SIMULATOR}
    plugins/robots/pi-puck/simulator/pipuck_omnidirectional_camera_default_sensor.cpp)
elseif(ARGOS_BUILD_FOR STREQUAL "pi-puck")
  set(ARGOS3_PIPUCK_OMNIDIRECTIONAL_CAMERA_SOURCES
    ${ARGOS3_PIPUCK_OMNIDIRECTIONAL_CAMERA_SOURCES}
    ${ARGOS3_PIPUCK_OMNIDIRECTIONAL_CAMERA_HEADERS_HARDWARE}
    plugins/robots/pi-puck/hardware/pipuck_omnidirectional_camera_default_sensor.cpp)
endif(ARGOS_BUILD_FOR_SIMULATOR)

if(ARGOS_BUILD_FOR_SIMULATOR)
  # Create a plug-in for the simulator
  add_library(argos3plugin_${ARGOS_BUILD_FOR}_pipuck_omnidirectional_camera SHARED ${ARGOS3_PIPUCK_OMNIDIRECTIONAL_CAMERA_SOURCES})
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_pipuck_omnidirectional_camera
    argos3core_${ARGOS_BUILD_FOR}   
    argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
    argos3plugin_${ARGOS_BUILD_FOR}_entities)
  install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_pipuck_omnidirectional_camera
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/argos3
    ARCHIVE DESTINATION lib/argos3)
elseif(ARGOS_BUILD_FOR STREQUAL "pi-puck")
  # Create a plugin for the robot
  add_library(argos3plugin_${ARGOS_BUILD_FOR}_pipuck_omnidirectional_camera SHARED ${ARGOS3_PIPUCK_OMNIDIRECTIONAL_CAMERA_SOURCES})
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_pipuck_omnidirectional_camera
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
    ${APRILTAG_LIBRARY})
  install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_pipuck_omnidirectional_camera
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/argos3
    ARCHIVE DESTINATION lib/argos3)
endif(ARGOS_BUILD_FOR_SIMULATOR)
install(FILES ${ARGOS3_PIPUCK_OMNIDIRECTIONAL_CAMERA_HEADERS_CONTROLINTERFACE} DESTINATION include/argos3/plugins/robots/pi-puck/control_interface)

# configure and install omnidirectional cameera test
configure_file(
  ${CMAKE_SOURCE_DIR}/testing/test_omnidirectional_camera.lua
  ${CMAKE_BINARY_DIR}/testing/test_omnidirectional_camera.lua)
if(ARGOS_BUILD_FOR_SIMULATOR)
  configure_file(
    ${CMAKE_SOURCE_DIR}/testing/test_omnidirectional_camera_simulator.argos.in
    ${CMAKE_BINARY_DIR}/testing/test_omnidirectional_camera.argos)
elseif(ARGOS_BUILD_FOR STREQUAL "pi-puck")
  configure_file(
    ${CMAKE_SOURCE_DIR}/testing/test_omnidirectional_camera_hardware.argos.in
    ${CMAKE_BINARY_DIR}/testing/test_omnidirectional_camera.argos)
  # install the tests to the robot image
  install(FILES ${CMAKE_BINARY_DIR}/testing/test_omnidirectional_camera.lua DESTINATION share/argos3/testing)
  install(FILES ${CMAKE_BINARY_DIR}/testing/test_omnidirectional_camera.argos DESTINATION share/argos3/testing)
endif(ARGOS_BUILD_FOR_SIMULATOR)




