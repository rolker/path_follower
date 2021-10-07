cmake_minimum_required(VERSION 2.8.3)
project(path_follower)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  actionlib
  base_local_planner
  dubins_curves
  genmsg
  geographic_visualization_msgs
  geometry_msgs
  geographic_msgs
  nav_core
  nav_msgs
  path_planner_common
  project11
  roscpp
)

add_action_files(DIRECTORY action FILES path_follower.action)
generate_messages(DEPENDENCIES actionlib_msgs geographic_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_follower_plugin
  CATKIN_DEPENDS geometry_msgs nav_core roscpp project11 geographic_visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/path_follower.cpp src/path_follower_action.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    )

add_executable(mpc_adapter_node src/mpc_adapter_node.cpp)

add_dependencies(mpc_adapter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(mpc_adapter_node
    ${catkin_LIBRARIES}
)

add_library(path_follower_plugin src/path_follower.cpp src/path_follower_plugin.cpp)
add_dependencies(path_follower_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_follower_plugin ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node path_follower_plugin
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
PATTERN ".svn" EXCLUDE
)

install(FILES path_follower_plugin.xml
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

