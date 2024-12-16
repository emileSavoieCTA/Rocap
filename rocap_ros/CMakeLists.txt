cmake_minimum_required(VERSION 3.5.0)
project(rocap_ros)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

link_directories(${gazebo_dev_LIBRARY_DIRS})

install(DIRECTORY
  launch
  config
  permissions
  urdf
  world
  DESTINATION share/${PROJECT_NAME}/
)

### generate action
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Move.action"
  "action/FollowPath.action"
  DEPENDENCIES geometry_msgs nav_msgs 
)

# gazebo_rope plugin
add_library(gazebo_rope SHARED
  plugins/src/gazebo_rope.cpp
)
target_include_directories(gazebo_rope PUBLIC plugins/include)
ament_target_dependencies(gazebo_rope
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
)

ament_export_libraries(gazebo_rope)

add_executable(planner3D 
               node/nav/planner3D.cpp
               node/nav/AStarPlanner.cpp
)
target_include_directories(planner3D PUBLIC node/nav/include)
ament_target_dependencies(planner3D 
"rclcpp" 
"octomap_msgs" 
"octomap" 
"nav_msgs"
)

if(NOT WIN32)
  if(NOT APPLE)
    set(
      AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_rocap_ros
      "prepend-non-duplicate;LD_LIBRARY_PATH;${GAZEBO_PLUGIN_PATH}")
  else()
    set(
      AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_rocap_ros
      "prepend-non-duplicate;DYLD_LIBRARY_PATH;${GAZEBO_PLUGIN_PATH}")
  endif()
endif()
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/plugins/env-hooks/gazebo_plugins.sh.in")

#Install cpp executable
install(TARGETS
        planner3D
        DESTINATION lib/${PROJECT_NAME})

## Install Python Node
install(PROGRAMS  
node/api_bridge/ros2_api_bridge.py
node/api_bridge/transformlistener.py
node/nav/localPlanner.py    
DESTINATION lib/${PROJECT_NAME} )

ament_package()

install(DIRECTORY plugins/include/
        DESTINATION include)

install(TARGETS
gazebo_rope

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
