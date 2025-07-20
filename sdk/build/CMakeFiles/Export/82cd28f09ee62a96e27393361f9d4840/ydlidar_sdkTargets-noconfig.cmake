#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ydlidar_sdk::ydlidar_driver" for configuration ""
set_property(TARGET ydlidar_sdk::ydlidar_driver APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ydlidar_sdk::ydlidar_driver PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "C;CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libydlidar_driver.a"
  )

list(APPEND _cmake_import_check_targets ydlidar_sdk::ydlidar_driver )
list(APPEND _cmake_import_check_files_for_ydlidar_sdk::ydlidar_driver "${_IMPORT_PREFIX}/lib/libydlidar_driver.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
