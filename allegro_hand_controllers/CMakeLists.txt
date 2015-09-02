cmake_minimum_required(VERSION 2.8.3)
project(allegro_hand_controllers)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  libpcan
  roscpp
  sensor_msgs
  allegro_hand_common
  bhand
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES allegro_hand_controllers
  CATKIN_DEPENDS allegro_hand_common bhand libpcan roscpp sensor_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare executables
add_executable(allegro_node_grasp src/allegro_node_grasp.cpp)
target_link_libraries(allegro_node_grasp ${catkin_LIBRARIES})
add_executable(allegro_node_pd src/allegro_node_pd.cpp)
target_link_libraries(allegro_node_pd ${catkin_LIBRARIES})
add_executable(allegro_node_velsat src/allegro_node_velsat.cpp)
target_link_libraries(allegro_node_velsat ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS allegro_node_grasp allegro_node_pd allegro_node_velsat
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
