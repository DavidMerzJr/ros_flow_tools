cmake_minimum_required(VERSION 3.0.2)
project(ros_flow_tools)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  shape_msgs
  std_msgs
  std_srvs
  tf2_ros
  visualization_msgs
)

find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ros_flow_tools
  CATKIN_DEPENDS
    pcl_ros
    roscpp
    shape_msgs
    std_msgs
    std_srvs
    tf2_ros
    visualization_msgs
  DEPENDS
)

###########
## Build ##
###########

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/ros_valve.cpp
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(translate_mesh_node
  src/translate_mesh_node.cpp
)
target_link_libraries(translate_mesh_node
  ${catkin_LIBRARIES}
)

add_executable(pose_adjuster_node
  src/pose_adjuster.cpp
)
target_link_libraries(pose_adjuster_node
  ${catkin_LIBRARIES}
)

add_executable(pointcloud_publisher
  src/pointcloud_publisher.cpp
)
target_link_libraries(pointcloud_publisher
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
install(TARGETS
    ${PROJECT_NAME}
    pointcloud_publisher
    pose_adjuster_node
    translate_mesh_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ros_flow_tools.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
