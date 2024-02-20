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
#find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlebot_skillset_interfaces REQUIRED)
find_package(turtlebot_skillset REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
#--------------------------------------------------

include_directories(include)

#--------------------------------------------------------------------

add_library(tb3_skillset_lib
  src/Node.cpp
)

target_link_libraries(tb3_skillset_lib
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${nav2_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${rclcpp_action_LIBRARIES}
  ${turtlebot_skillset_interfaces_LIBRARIES}
  ${turtlebot_skillset_LIBRARIES}
)

ament_target_dependencies(tb3_skillset_lib 
  rclcpp 
  std_msgs  
  rclcpp_action
  nav2_msgs
  geometry_msgs
  turtlebot_skillset_interfaces 
  turtlebot_skillset)

install(DIRECTORY include/
  DESTINATION include)

install(
  TARGETS tb3_skillset_lib
  EXPORT tb3_skillset_lib
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

target_link_libraries(tb3_skillset_debug
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${nav2_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${rclcpp_action_LIBRARIES}
  ${turtlebot_skillset_interfaces_LIBRARIES}
  ${turtlebot_skillset_LIBRARIES}
)

target_compile_definitions(tb3_skillset_debug PUBLIC SKILLSET_DEBUG_MODE)
target_link_libraries(tb3_skillset_debug)

ament_target_dependencies(tb3_skillset_debug 
  rclcpp 
  std_msgs 
  rclcpp_action
  nav2_msgs
  geometry_msgs
  turtlebot_skillset_interfaces 
  turtlebot_skillset)

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
  tb3_skillset_lib
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

set(PROJECT_LIBS tb3_skillset_lib)

ament_export_targets(tb3_skillset_lib)

#--------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
