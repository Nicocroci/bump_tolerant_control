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
  BumpTolerantControl
  ExternalWrenchEstimator
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

## Generate dynamic parameters which are loaded by the dynamic reconfigure server
generate_dynamic_reconfigure_options(
  config/dynparam.cfg
  )

catkin_package(
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  DEPENDS Eigen
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )
## Declare a C++ library
add_library(BumpTolerantControl
  src/bump_tolerant_control.cpp
  )

## Add configure headers for dynamic reconfigure
add_dependencies(BumpTolerantControl
  ${PROJECT_NAME}_gencfg
  )

## Specify libraries to link a library or executable target against
target_link_libraries(BumpTolerantControl
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## Declare the ExternalWrenchEstimator nodelet library
add_library(ExternalWrenchEstimator
  src/external_wrench_estimator.cpp
  )

target_link_libraries(ExternalWrenchEstimator
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
