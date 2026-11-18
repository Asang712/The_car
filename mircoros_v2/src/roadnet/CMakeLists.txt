cmake_minimum_required(VERSION 3.5)
project(roadnet)

# 强制使用系统共享库版本的 tinyxml2，避免链接 /usr/local/lib/libtinyxml2.a（非 PIC 静态库）
set(TINYXML2_LIBRARY /usr/lib/x86_64-linux-gnu/libtinyxml2.so CACHE FILEPATH "" FORCE)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_util REQUIRED)
find_package(angles REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(nanoflann REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)

nav2_package()

set(action_files
  "interfaces/action/ComputeRoute.action"
  "interfaces/action/ComputeAndTrackRoute.action"
  "interfaces/action/RoadNetToGoal.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs action_msgs nav_msgs nav2_msgs
)

ament_export_dependencies(rosidl_default_runtime)

set(executable_name route_server)
set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  geometry_msgs
  nav2_msgs
  nav2_costmap_2d
  pluginlib
  visualization_msgs
  nav_msgs
  tf2_ros
  nav2_core
  nav2_util
  angles
  nanoflann
  nlohmann_json
  rosidl_default_runtime
  rosidl_typesupport_cpp
)

include_directories(
  include
)

# Qt5 setup for route_tool
set(route_tool_headers_to_moc
  include/roadnet/route_tool.hpp
)

foreach(header "${route_tool_headers_to_moc}")
  qt5_wrap_cpp(route_tool_moc_files "${header}")
endforeach()

qt5_wrap_ui(route_tool_UIS_H resource/route_tool.ui)

# Qt5 setup for roadnet_nav_tool
set(roadnet_nav_tool_headers_to_moc
  include/roadnet/roadnet_nav_tool.hpp
)

foreach(header "${roadnet_nav_tool_headers_to_moc}")
  qt5_wrap_cpp(roadnet_nav_tool_moc_files "${header}")
endforeach()

qt5_wrap_ui(roadnet_nav_tool_UIS_H resource/roadnet_nav_tool.ui)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/resource)

# Main library
add_library(${library_name} SHARED
  src/route_server.cpp
  src/route_planner.cpp
  src/route_tracker.cpp
  src/edge_scorer.cpp
  src/operations_manager.cpp
  src/node_spatial_tree.cpp
  src/path_converter.cpp
  src/graph_loader.cpp
  src/graph_saver.cpp
  src/goal_intent_extractor.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${library_name}
  ${PROJECT_NAME}__rosidl_generator_cpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Main executable
add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${executable_name}
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "roadnet::RouteServer")

# Edge scoring plugins
add_library(edge_scorers SHARED
  src/plugins/edge_cost_functions/distance_scorer.cpp
  src/plugins/edge_cost_functions/time_scorer.cpp
  src/plugins/edge_cost_functions/dynamic_edges_scorer.cpp
  src/plugins/edge_cost_functions/penalty_scorer.cpp
  src/plugins/edge_cost_functions/costmap_scorer.cpp
  src/plugins/edge_cost_functions/semantic_scorer.cpp
  src/plugins/edge_cost_functions/goal_orientation_scorer.cpp
  src/plugins/edge_cost_functions/start_pose_orientation_scorer.cpp
)

ament_target_dependencies(edge_scorers
  ${dependencies}
)

target_link_libraries(edge_scorers
  ${PROJECT_NAME}__rosidl_generator_cpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Route operations plugins
add_library(route_operations SHARED
  src/plugins/route_operations/adjust_speed_limit.cpp
  src/plugins/route_operations/trigger_event.cpp
  src/plugins/route_operations/rerouting_service.cpp
  src/plugins/route_operations/collision_monitor.cpp
  src/plugins/route_operations/time_marker.cpp
)

ament_target_dependencies(route_operations
  ${dependencies}
)

target_link_libraries(route_operations
  ${PROJECT_NAME}__rosidl_generator_cpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Graph Parser plugins
add_library(graph_file_loaders SHARED
    src/plugins/graph_file_loaders/geojson_graph_file_loader.cpp
)

ament_target_dependencies(graph_file_loaders
    ${dependencies}
)

target_link_libraries(graph_file_loaders
  ${PROJECT_NAME}__rosidl_generator_cpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

add_library(graph_file_savers SHARED
    src/plugins/graph_file_savers/geojson_graph_file_saver.cpp
)

ament_target_dependencies(graph_file_savers
    ${dependencies}
)

target_link_libraries(graph_file_savers
  ${PROJECT_NAME}__rosidl_generator_cpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Route Tool (rviz panel)
set(route_tool_name route_tool)
add_library(${route_tool_name} SHARED
  src/route_tool.cpp
  ${route_tool_moc_files}
  ${route_tool_UIS_H}
)

target_include_directories(${route_tool_name} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(${route_tool_name}
  ${dependencies}
  rviz_common
  rviz_default_plugins
  rviz_rendering
)

target_link_libraries(${route_tool_name} ${library_name} Qt5::Core Qt5::Gui Qt5::Widgets)

pluginlib_export_plugin_description_file(rviz_common route_tool_rviz.xml)

# Roadnet Nav Tool (rviz panel)
set(roadnet_nav_tool_name roadnet_nav_tool)
add_library(${roadnet_nav_tool_name} SHARED
  src/roadnet_nav_tool.cpp
  ${roadnet_nav_tool_moc_files}
  ${roadnet_nav_tool_UIS_H}
)

target_include_directories(${roadnet_nav_tool_name} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(${roadnet_nav_tool_name}
  ${dependencies}
  rviz_common
  rviz_default_plugins
  rviz_rendering
)

target_link_libraries(${roadnet_nav_tool_name} ${library_name} Qt5::Core Qt5::Gui Qt5::Widgets)

pluginlib_export_plugin_description_file(rviz_common roadnet_nav_tool_rviz.xml)

pluginlib_export_plugin_description_file(roadnet plugins.xml)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${library_name} edge_scorers route_operations graph_file_loaders graph_file_savers ${route_tool_name} ${roadnet_nav_tool_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY resource DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  graphs  
  icons
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY test/test_graphs DESTINATION share/${PROJECT_NAME}/test)

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   add_subdirectory(test)
# endif()

ament_export_include_directories(include)
ament_export_dependencies(
  ${dependencies}
  Qt5
  rviz_common
  rviz_default_plugins
  rviz_rendering
)
ament_export_libraries(${library_name} edge_scorers route_operations graph_file_loaders ${route_tool_name} ${roadnet_nav_tool_name})
ament_package()
