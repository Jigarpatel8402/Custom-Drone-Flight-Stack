#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "drone_interfaces::drone_interfaces__rosidl_generator_py" for configuration "Debug"
set_property(TARGET drone_interfaces::drone_interfaces__rosidl_generator_py APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(drone_interfaces::drone_interfaces__rosidl_generator_py PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_DEBUG "drone_interfaces::drone_interfaces__rosidl_generator_c;Python3::Python;drone_interfaces::drone_interfaces__rosidl_typesupport_c"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libdrone_interfaces__rosidl_generator_py.so"
  IMPORTED_SONAME_DEBUG "libdrone_interfaces__rosidl_generator_py.so"
  )

list(APPEND _cmake_import_check_targets drone_interfaces::drone_interfaces__rosidl_generator_py )
list(APPEND _cmake_import_check_files_for_drone_interfaces::drone_interfaces__rosidl_generator_py "${_IMPORT_PREFIX}/lib/libdrone_interfaces__rosidl_generator_py.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
