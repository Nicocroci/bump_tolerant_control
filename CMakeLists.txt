cmake_minimum_required(VERSION 3.15.0)
project(bump_tolerant_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CATKIN_DEPENDENCIES
  cmake_modules
  roscpp
  rospy
  nodelet
  pluginlib
  geometry_msgs
  nav_msgs
  mrs_msgs
  mrs_lib
  mrs_uav_managers
  dynamic_reconfigure
  sensor_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen3_LIBRARIES})

generate_dynamic_reconfigure_options(
  cfg/bump_tolerant_controller.cfg
  cfg/dynparam_example_waypoint_flier.cfg
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

set(LIBRARIES
  ExternalWrenchEstimator
  BumpSwitchNode
  #ExampleWaypointFlier
  BumpTolerantController
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  #DEPENDS Eigen
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

## Declare the nodelet libraries
add_library(ExternalWrenchEstimator
  src/external_wrench_estimator.cpp
  )

add_dependencies(ExternalWrenchEstimator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(ExternalWrenchEstimator
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(BumpSwitchNode
  src/bump_switch_node.cpp
  )

add_dependencies(BumpSwitchNode
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(BumpSwitchNode
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

#add_library(ExampleWaypointFlier
#  src/example_waypoint_flier.cpp
#  )

#add_dependencies(ExampleWaypointFlier
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#  ${catkin_EXPORTED_TARGETS}
# ${PROJECT_NAME}_gencfg
#  )

#target_link_libraries(ExampleWaypointFlier
#  ${catkin_LIBRARIES}
#  ${Eigen_LIBRARIES}
#  )
  
add_library(BumpTolerantController
  src/bump_tolerant_controller.cpp
)

add_dependencies(BumpTolerantController
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(BumpTolerantController
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

## Executable for testing
add_executable(imu_listener
  src/imu_listener.cpp
  )

add_dependencies(imu_listener
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(imu_listener
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(TARGETS imu_listener
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES plugins.xml nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )