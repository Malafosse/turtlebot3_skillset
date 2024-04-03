cmake_minimum_required(VERSION 3.5)
project(tb3_skillset)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-stdlib=libstdc++)
endif()

#--------------------------------------------------
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(turtlebot_skillset_interfaces REQUIRED)
find_package(turtlebot_skillset REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

#find_package(unique_identifier_msgs REQUIRED)
#--------------------------------------------------

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${nav2_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${diagnostic_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_msgs_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
 # ${unique_identifier_msgs_INCLUDE_DIRS}
  ${turtlebot_skillset_interfaces_INCLUDE_DIRS}
  ${turtlebot_skillset_INCLUDE_DIRS}
)

#--------------------------------------------------------------------

add_library(tb3_skillset
  src/Node.cpp
  
)

target_link_libraries(tb3_skillset
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${rclcpp_action_LIBRARIES}
  ${nav2_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_msgs_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  #${unique_identifier_msgs_LIBRARIES}
  ${diagnostic_msgs_LIBRARIES}
  ${turtlebot_skillset_interfaces_LIBRARIES}
  ${turtlebot_skillset_LIBRARIES}
)

ament_target_dependencies(tb3_skillset 
  rclcpp 
  std_msgs 
  rclcpp_action
  turtlebot_skillset_interfaces 
  turtlebot_skillset
  nav2_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_msgs
  image_transport
  cv_bridge
  diagnostic_msgs
)

install(DIRECTORY include/
  DESTINATION include)

install(
  TARGETS tb3_skillset
  EXPORT tb3_skillset
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#--------------------------------------------------------------------
# DEBUG

add_library(tb3_skillset_debug
  src/Node.cpp
)


target_compile_definitions(tb3_skillset_debug PUBLIC SKILLSET_DEBUG_MODE)
target_link_libraries(tb3_skillset_debug
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${rclcpp_action_LIBRARIES}
  ${nav2_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_msgs_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  #${unique_identifier_msgs_LIBRARIES}
  ${diagnostic_msgs_LIBRARIES}
  ${turtlebot_skillset_interfaces_LIBRARIES}
  ${turtlebot_skillset_LIBRARIES}
)
ament_target_dependencies(tb3_skillset_debug 
  rclcpp 
  std_msgs 
  rclcpp_action
  turtlebot_skillset_interfaces 
  turtlebot_skillset
  nav2_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_msgs
  image_transport
  cv_bridge
  diagnostic_msgs
)

install(
  TARGETS tb3_skillset_debug
  EXPORT tb3_skillset_debug
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#--------------------------------------------------------------------

add_executable(tb3_skillset_node src/tb3_skillset_node.cpp)
target_link_libraries(tb3_skillset_node
  tb3_skillset
)
install(TARGETS
  tb3_skillset_node
  DESTINATION lib/${PROJECT_NAME})

#--------------------------------------------------------------------
# DEBUG

add_executable(tb3_skillset_node_debug src/tb3_skillset_node.cpp)
target_link_libraries(tb3_skillset_node_debug
  tb3_skillset_debug
)
install(TARGETS
  tb3_skillset_node_debug
  DESTINATION lib/${PROJECT_NAME})

#--------------------------------------------------------------------

set(PROJECT_LIBS tb3_skillset)

ament_export_targets(tb3_skillset)

#--------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
