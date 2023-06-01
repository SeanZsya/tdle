cmake_minimum_required(VERSION 2.8.3)
project(tdle)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  actionlib
  visualization_msgs
  message_generation
)

add_message_files(
  FILES
  PointArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package( 
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(
  fpoint_detector 
  src/fpoint_detector_node.cpp  
  src/common_func.cpp)
target_link_libraries(fpoint_detector ${catkin_LIBRARIES})
add_dependencies(fpoint_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(
  fpoint_filter
  src/fpoint_filter_node.cpp
  src/common_func.cpp)
target_link_libraries(fpoint_filter ${catkin_LIBRARIES})
add_dependencies(fpoint_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(
  subregion_arrangement
  src/subregion_arrangement_node.cpp
  src/common_func.cpp
  )
target_link_libraries(subregion_arrangement ${catkin_LIBRARIES})
add_dependencies(subregion_arrangement ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(
  fpoint_selection
  src/fpoint_selection_node.cpp
  src/common_func.cpp)
target_link_libraries(fpoint_selection ${catkin_LIBRARIES})
add_dependencies(fpoint_selection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(
  data_recorder
  src/data_recorder_node.cpp
  src/common_func.cpp)
target_link_libraries(data_recorder ${catkin_LIBRARIES})
add_dependencies(data_recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})