#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug -DCMAKE_C_FLAGS_DEBUG="-g -O0" -DCMAKE_CXX_FLAGS_DEBUG="-g -O0"".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "glfw" for configuration "Debug -DCMAKE_C_FLAGS_DEBUG="-g -O0" -DCMAKE_CXX_FLAGS_DEBUG="-g -O0""
set_property(TARGET glfw APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG -DCMAKE_C_FLAGS_DEBUG="-G -O0" -DCMAKE_CXX_FLAGS_DEBUG="-G -O0")
set_target_properties(glfw PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG -DCMAKE_C_FLAGS_DEBUG="-G -O0" -DCMAKE_CXX_FLAGS_DEBUG="-G -O0" "C"
  IMPORTED_LOCATION_DEBUG -DCMAKE_C_FLAGS_DEBUG="-G -O0" -DCMAKE_CXX_FLAGS_DEBUG="-G -O0" "${_IMPORT_PREFIX}/lib/libglfw3.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS glfw )
list(APPEND _IMPORT_CHECK_FILES_FOR_glfw "${_IMPORT_PREFIX}/lib/libglfw3.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
