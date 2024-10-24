cmake_minimum_required(VERSION 3.5)
project(pinpoint)

# find dependencies
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Name build targets
set(node_exec pinpoint_node_exec)
set(node_lib pinpoint_node)

# Includes
include_directories(
  include
)

ament_export_include_directories(include)

ament_auto_add_library(${node_lib} SHARED
  src/pinpoint_application.cpp
)

ament_auto_add_executable(${node_exec}
  src/main.cpp
)

# Register component
rclcpp_components_register_nodes(${node_lib} "pinpoint::PinPointApplication")

# All locally created targets will need to be manually linked
# ament auto will handle linking of external dependencies
target_link_libraries(${node_exec}
  ${node_lib}
)

ament_auto_package(
  INSTALL_TO_SHARE config launch
)
