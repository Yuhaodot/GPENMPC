include("${CMAKE_CURRENT_LIST_DIR}/../../../PreparedPaths.cmake")

if(NOT GPENMPC_BOARD_UID MATCHES "^[1-9][0-9]*$")
  message(FATAL_ERROR "Set GPENMPC_BOARD_UID to the expected nonzero decimal board UID")
endif()
string(LENGTH "${GPENMPC_BOARD_UID}" _uid_length)
if(_uid_length GREATER 20 OR (_uid_length EQUAL 20 AND GPENMPC_BOARD_UID STRGREATER "18446744073709551615"))
  message(FATAL_ERROR "GPENMPC_BOARD_UID exceeds unsigned 64-bit range")
endif()

# Serialized NSH entry; the owner validates the physical declaration.
px4_add_module(
  MODULE modules__gpenmpc_rfly_session
  MAIN gpenmpc_rfly_session
  # NSH command-task stack; the controller task has its own allocation.
  STACK_MAIN 4096
  SRCS "${GPENMPC_RUNTIME}/CanonicalLocalSessionEntry.cpp"
  INCLUDES "${GPENMPC_MAVLINK_OVERLAY}" "${GPENMPC_GENERATED}"
    "${GPENMPC_RFLY}/official_io_nuttx/overlay" "${GPENMPC_INGRESS}"
    "${GPENMPC_RFLY}/local_application_profile" "${GPENMPC_RFLY}/local_source_ingress"
  COMPILE_FLAGS -ffp-contract=off -fno-fast-math -fstack-usage -DGPENMPC_CANONICAL_CLOSED_EVIDENCE=1
  DEPENDS gpenmpc_rfly_runtime_components modules__gpenmpc_rfly_canonical)
target_compile_definitions(modules__gpenmpc_rfly_session PRIVATE GPENMPC_BOARD_UID=${GPENMPC_BOARD_UID}ULL)
target_include_directories(modules__gpenmpc_rfly_session SYSTEM PRIVATE
  "${GPENMPC_INGRESS}/nuttx_include_overlay")
