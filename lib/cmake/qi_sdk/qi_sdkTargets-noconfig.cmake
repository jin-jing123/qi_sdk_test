#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "qi_sdk" for configuration ""
set_property(TARGET qi_sdk APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(qi_sdk PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/x86_64/libqi_sdk.a"
  )

list(APPEND _cmake_import_check_targets qi_sdk )
list(APPEND _cmake_import_check_files_for_qi_sdk "${_IMPORT_PREFIX}/lib/x86_64/libqi_sdk.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
