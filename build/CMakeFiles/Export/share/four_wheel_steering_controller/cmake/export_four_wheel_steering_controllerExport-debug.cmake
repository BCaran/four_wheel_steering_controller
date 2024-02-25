#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "four_wheel_steering_controller::four_wheel_steering_controller" for configuration "Debug"
set_property(TARGET four_wheel_steering_controller::four_wheel_steering_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(four_wheel_steering_controller::four_wheel_steering_controller PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libfour_wheel_steering_controller.so"
  IMPORTED_SONAME_DEBUG "libfour_wheel_steering_controller.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS four_wheel_steering_controller::four_wheel_steering_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_four_wheel_steering_controller::four_wheel_steering_controller "${_IMPORT_PREFIX}/lib/libfour_wheel_steering_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
