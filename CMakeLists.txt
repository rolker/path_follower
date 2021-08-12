cmake_minimum_required(VERSION 2.8.3)
project(path_follower)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  geographic_msgs
  roscpp
  project11
  genmsg
  actionlib_msgs
  actionlib
  geographic_visualization_msgs
  path_planner_common
  dubins_curves
  nav_msgs
)

add_action_files(DIRECTORY action FILES path_follower.action)
generate_messages(DEPENDENCIES actionlib_msgs geographic_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES asv_helm
  CATKIN_DEPENDS geometry_msgs roscpp project11 geographic_visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/path_follower_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    )

add_executable(holonomic_follower_node src/holonomic_follower_node.cpp)
add_dependencies(holonomic_follower_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(holonomic_follower_node
    ${catkin_LIBRARIES}
)

add_executable(mpc_adapter_node src/mpc_adapter_node.cpp)

add_dependencies(mpc_adapter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(mpc_adapter_node
    ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

