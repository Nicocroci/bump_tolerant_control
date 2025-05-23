cmake_minimum_required(VERSION 3.15.0)
project(bump_tolerant_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Uncomment these two lines to enable debugging symbols and disable optimization for debugging purposes.
# add_definitions(-g)
# add_definitions(-O0)

SET(CATKIN_DEPENDENCIES
  roscpp
  nodelet
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  mrs_msgs
  mrs_lib
  sensor_msgs
  )

SET(LIBRARIES
  ExternalWrenchEstimator
  BumpSwitchNode
  ExampleWaypointFlier
  BumpTolerantController
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

## Generate dynamic parameters which are loaded by the dynamic reconfigure server
generate_dynamic_reconfigure_options(
  config/bump_tolerant_controller.cfg
  config/dynparam_example_waypoint_flier.cfg
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  DEPENDS Eigen
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )


## Declare the nodelet libraries
add_library(ExternalWrenchEstimator
  src/external_wrench_estimator.cpp
  )

target_link_libraries(ExternalWrenchEstimator
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(BumpSwitchNode
  src/bump_switch_node.cpp
  )

target_link_libraries(BumpSwitchNode
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(ExampleWaypointFlier
  src/example_waypoint_flier.cpp
  )

add_dependencies(ExampleWaypointFlier
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(ExampleWaypointFlier
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )
  
add_library(BumpTolerantController
  src/bump_tolerant_controller.cpp
)

target_link_libraries(BumpTolerantController
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)
## Declare a C++ executable
add_executable(imu_listener
  src/imu_listener.cpp
  )

target_link_libraries(imu_listener
  ${catkin_LIBRARIES}
  )
## Add configure headers for dynamic reconfigure
## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)