#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "blasfeo" for configuration "Release"
set_property(TARGET blasfeo APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(blasfeo PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libblasfeo.so.0.1.4.2"
  IMPORTED_SONAME_RELEASE "libblasfeo.so.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS blasfeo )
list(APPEND _IMPORT_CHECK_FILES_FOR_blasfeo "${_IMPORT_PREFIX}/lib/libblasfeo.so.0.1.4.2" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
