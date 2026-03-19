cmake_minimum_required(VERSION 3.8)
project(tdle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PointArray.msg"
  DEPENDENCIES geometry_msgs
)

include_directories(include)

add_executable(fpoint_detector
  src/fpoint_detector_node.cpp
  src/common_func.cpp
)
ament_target_dependencies(fpoint_detector
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

add_executable(fpoint_filter
  src/fpoint_filter_node.cpp
  src/common_func.cpp
)
ament_target_dependencies(fpoint_filter
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

add_executable(subregion_arrangement
  src/subregion_arrangement_node.cpp
  src/common_func.cpp
)
ament_target_dependencies(subregion_arrangement
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

add_executable(fpoint_selection
  src/fpoint_selection_node.cpp
  src/common_func.cpp
)
ament_target_dependencies(fpoint_selection
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  nav2_msgs
)

add_executable(data_recorder
  src/data_recorder_node.cpp
  src/common_func.cpp
)
ament_target_dependencies(data_recorder
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

# Get typesupport target for linking
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Link custom message interfaces
target_link_libraries(fpoint_detector ${cpp_typesupport_target})
target_link_libraries(fpoint_filter ${cpp_typesupport_target})
target_link_libraries(subregion_arrangement ${cpp_typesupport_target})
target_link_libraries(fpoint_selection ${cpp_typesupport_target})
target_link_libraries(data_recorder ${cpp_typesupport_target})

install(TARGETS
  fpoint_detector
  fpoint_filter
  subregion_arrangement
  fpoint_selection
  data_recorder
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY param/
  DESTINATION share/${PROJECT_NAME}/param
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
