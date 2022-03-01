#
# Check ARGoS
#
find_package(ARGoS REQUIRED)
include_directories(${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIR})
link_libraries(${ARGOS_LDFLAGS})

if(ARGOS_BUILD_FOR STREQUAL "pi-puck")
  find_package(AprilTag)
  if(NOT APRILTAG_FOUND)
    message(FATAL_ERROR "Required library AprilTag not found.")
  endif(NOT APRILTAG_FOUND)
  include_directories(${APRILTAG_INCLUDE_DIR})
endif(ARGOS_BUILD_FOR STREQUAL "pi-puck")
