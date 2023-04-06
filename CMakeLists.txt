cmake_minimum_required(VERSION 2.8.3)
project(path_follower)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geographic_visualization_msgs
  geometry_msgs
  geographic_msgs
  nav_msgs
  project11
  project11_navigation
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_follower_plugin
  CATKIN_DEPENDS geometry_msgs roscpp project11 project11_navigation  geographic_visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(path_follower_plugin src/path_follower.cpp)
add_dependencies(path_follower_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_follower_plugin ${catkin_LIBRARIES})

install(TARGETS path_follower_plugin
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

