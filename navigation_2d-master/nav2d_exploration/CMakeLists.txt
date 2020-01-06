cmake_minimum_required(VERSION 2.8.3)
project(nav2d_exploration)
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  pluginlib
  geometry_msgs
  nav_msgs
  visualization_msgs
  nav2d_navigator
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Log.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  nav_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES ExplorationPlugins
  CATKIN_DEPENDS
    roscpp
	tf
	pluginlib
	geometry_msgs
	nav_msgs
	visualization_msgs
	nav2d_navigator
    message_runtime
)

###########
## Build ##
###########

include_directories(
  include/${PROJECT_NAME}/
  include/${PROJECT_NAME}/BlueprintExploration
  ${catkin_INCLUDE_DIRS}
)
add_library(ExplorationPlugins
  src/NearestFrontierPlanner.cpp
  src/MultiWavefrontPlanner.cpp
  src/MinPosPlanner.cpp
  src/exploration_plugins.cpp
  src/BlueprintExploration.cpp
  src/BlueprintExploration/FloorPlan.cpp
  src/BlueprintExploration/FrontierListHandler.cpp
  src/BlueprintExploration/FrontierHandler.cpp
  src/BlueprintExploration/PoseMonitor.cpp
  src/BlueprintExploration/Logger.cpp
  src/BlueprintExploration/CoverageComputer.cpp
)

add_dependencies(ExplorationPlugins ${catkin_EXPORTED_TARGETS})

target_link_libraries(ExplorationPlugins ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS ExplorationPlugins
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  exploration.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  nodes/logger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nav2d_exploration.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
