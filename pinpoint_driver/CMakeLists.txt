cmake_minimum_required(VERSION 3.5)

project(pinpoint_driver)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# Includes
include_directories(
  include
)

ament_export_include_directories(include)

## Declare a C++ library
ament_auto_add_library(pinpoint_driver SHARED
        src/pinpoint_driver.cpp
        src/pinpoint_localization_client.cpp
)

# Install
ament_auto_package(
        INSTALL_TO_SHARE
)
