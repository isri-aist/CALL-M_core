#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "zed_interfaces::zed_interfaces__rosidl_typesupport_cpp" for configuration "Release"
set_property(TARGET zed_interfaces::zed_interfaces__rosidl_typesupport_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(zed_interfaces::zed_interfaces__rosidl_typesupport_cpp PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libzed_interfaces__rosidl_typesupport_cpp.so"
  IMPORTED_SONAME_RELEASE "libzed_interfaces__rosidl_typesupport_cpp.so"
  )

list(APPEND _cmake_import_check_targets zed_interfaces::zed_interfaces__rosidl_typesupport_cpp )
list(APPEND _cmake_import_check_files_for_zed_interfaces::zed_interfaces__rosidl_typesupport_cpp "${_IMPORT_PREFIX}/lib/libzed_interfaces__rosidl_typesupport_cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
