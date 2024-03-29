cmake_minimum_required(VERSION 3.16)
project(four_wheel_steering_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(four_wheel_steering_controller_parameters
  src/four_wheel_steering_controller_parameter.yaml
)

add_library(four_wheel_steering_controller SHARED
  src/four_wheel_steering_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)
target_compile_features(four_wheel_steering_controller PUBLIC cxx_std_17)
target_include_directories(four_wheel_steering_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/four_wheel_steering_controller>
)
target_link_libraries(four_wheel_steering_controller PUBLIC four_wheel_steering_controller_parameters)
ament_target_dependencies(four_wheel_steering_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(four_wheel_steering_controller PRIVATE "four_wheel_steering_controller_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface four_wheel_steering_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_four_wheel_steering_controller
    test/test_four_wheel_steering_controller.cpp
    ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_four_wheel_steering_controller.yaml)
  target_link_libraries(test_four_wheel_steering_controller
    four_wheel_steering_controller
  )
  ament_target_dependencies(test_four_wheel_steering_controller
    geometry_msgs
    hardware_interface
    nav_msgs
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    tf2
    tf2_msgs
  )

  ament_add_gmock(test_load_four_wheel_steering_controller
    test/test_load_four_wheel_steering_controller.cpp
  )
  ament_target_dependencies(test_load_four_wheel_steering_controller
    controller_manager
    ros2_control_test_assets
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/four_wheel_steering_controller
)
install(TARGETS four_wheel_steering_controller four_wheel_steering_controller_parameters
  EXPORT export_four_wheel_steering_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_four_wheel_steering_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
