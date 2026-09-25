# Redirect the writable board source path while retaining CMake caller scope.
macro(add_subdirectory)
  get_filename_component(_gpenmpc_exact_source "${ARGV0}" ABSOLUTE
    BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
  if(_gpenmpc_exact_source STREQUAL "${PX4_BOARD_DIR}/src" AND ${ARGC} EQUAL 1)
    _add_subdirectory("${ARGV0}"
      "${PX4_BINARY_DIR}/boards/px4/fmu-v6c/src")
  else()
    _add_subdirectory(${ARGV})
  endif()
endmacro()
