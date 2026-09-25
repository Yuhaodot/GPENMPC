cmake_minimum_required(VERSION 3.19)
include("${CMAKE_CURRENT_LIST_DIR}/../../PreparedPaths.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../PrivateExternalProjects.cmake")

# Parameter-only compatibility: Commander and the native-LAND safety guard
# still require the original RA_CTRL_MODE Int32/default 0 metadata. Keep the
# original directory in PX4's parameter scan without linking its retired
# mode-1/mode-2 controller or adding a CLI/startup path.
set_property(GLOBAL APPEND PROPERTY PX4_MODULE_PATHS
  "${PX4_SOURCE_DIR}/src/modules/gpenmpc_se3_control"
  "${PX4_SOURCE_DIR}/src/modules/gpenmpc_trajectory_exec")

# ModuleBase CLI uses the application-supplied context.
set(config_module_list_external "gpenmpc_rfly_canonical;gpenmpc_rfly_session" PARENT_SCOPE)
add_library(gpenmpc_rfly_runtime_components STATIC EXCLUDE_FROM_ALL
  "${GPENMPC_RUNTIME}/Px4CanonicalLocalIo.cpp"
  "${GPENMPC_RUNTIME}/Px4ReadOnlyGuard.cpp"
  "${GPENMPC_RUNTIME}/Px4ExecutionSessionGuard.cpp"
  "${GPENMPC_RUNTIME}/ExecutionSessionRegistration.cpp"
  "${GPENMPC_RUNTIME}/CanonicalLocalExecutionCycle.cpp"
  "${GPENMPC_RUNTIME}/CanonicalLocalExchangeCore.cpp"
  "${GPENMPC_RUNTIME}/CanonicalLocalWireOutbox.cpp"
  "${GPENMPC_RUNTIME}/CanonicalLocalTaskInputOwner.cpp"
  "${GPENMPC_RUNTIME}/RegisteredCanonicalLocalModuleContext.cpp"
  "${GPENMPC_RUNTIME}/CanonicalLocalApplicationOwner.cpp"
  "${GPENMPC_RFLY}/local_source_ingress/Px4OriginalHilReceiptReader.cpp"
  "${GPENMPC_RFLY}/local_source_ingress/Px4SelectedSourceReader.cpp"
  "${GPENMPC_RFLY}/px4_wire/CanonicalLocalWindowAssembler.cpp"
  "${GPENMPC_RFLY}/px4_wire/CanonicalLocalWindowWire.cpp"
  "${GPENMPC_RUNTIME}/LegacyTrajectoryReadOnly.cpp"
  "${GPENMPC_RFLY}/px4_stream/SharedOutputRegistry.cpp"
  "${GPENMPC_RFLY}/px4_stream/LinkLifetimeRegistry.cpp"
  "${GPENMPC_RFLY}/px4_stream/LinkCanonicalReservation.cpp"
  "${GPENMPC_RFLY}/px4_stream/CanonicalLocalWireRouteRegistry.cpp")
include("${CMAKE_CURRENT_LIST_DIR}/../../KernelLibrary.cmake")
add_library(gpenmpc_full_inner_facade STATIC EXCLUDE_FROM_ALL
  "${GPENMPC_RFLY}/full_inner_abi/CanonicalFullInnerAbi.cpp"
  "${GPENMPC_RFLY}/../evidence/phase_controller/generated/gpenmpcNative_canonicalLocalPhaseAdvance.c")
target_compile_definitions(gpenmpc_full_inner_facade PRIVATE GPENMPC_CANONICAL_EXPLICIT_WORKSPACE=1 GPENMPC_CANONICAL_CLOSED_EVIDENCE=1 GPENMPC_CANONICAL_LEARNING_AUDIT=1 GPENMPC_OPERATOR_YAW_REFERENCE=1 MODULE_NAME="gpenmpc_full_inner")
target_compile_options(gpenmpc_full_inner_facade PRIVATE -ffp-contract=off -fno-fast-math -fstack-usage)
target_include_directories(gpenmpc_full_inner_facade BEFORE PRIVATE
  "${GPENMPC_IDENTITY_DIR}"
  "${GPENMPC_RFLY}/../evidence/tracking_controller/generated"
  "${GPENMPC_RFLY}/official_io_nuttx/overlay" "${GPENMPC_INGRESS}")
if(NOT EXISTS "${GPENMPC_MATLAB_INCLUDE}/tmwtypes.h")
  message(FATAL_ERROR "Set GPENMPC_MATLAB_INCLUDE to the MATLAB extern/include directory")
endif()
target_include_directories(gpenmpc_full_inner_facade SYSTEM PRIVATE
  "${GPENMPC_INGRESS}/nuttx_include_overlay" "${GPENMPC_MATLAB_INCLUDE}")
target_link_libraries(gpenmpc_full_inner_facade PUBLIC gpenmpc_full_inner_private)
add_dependencies(gpenmpc_full_inner_facade gpenmpc_build_identity)
target_link_libraries(gpenmpc_rfly_runtime_components PRIVATE gpenmpc_full_inner_facade)
target_include_directories(gpenmpc_rfly_runtime_components BEFORE PRIVATE
  "${GPENMPC_MAVLINK_OVERLAY}" "${GPENMPC_GENERATED}"
  "${GPENMPC_RFLY}/official_io_nuttx/overlay"
  "${GPENMPC_RFLY}/local_source_ingress"
  "${GPENMPC_INGRESS}")
target_include_directories(gpenmpc_rfly_runtime_components SYSTEM PRIVATE
  "${GPENMPC_INGRESS}/nuttx_include_overlay")
target_compile_options(gpenmpc_rfly_runtime_components PRIVATE -ffp-contract=off -fno-fast-math -fstack-usage)
target_compile_definitions(gpenmpc_rfly_runtime_components PUBLIC GPENMPC_CANONICAL_CLOSED_EVIDENCE=1 GPENMPC_CANONICAL_LEARNING_AUDIT=1 PRIVATE
  MODULE_NAME="gpenmpc_rfly_components")

# At this point modules__mavlink and the drivers have not yet been declared.
# Read paths from this file again at root-scope deferred execution, not from
# variables local to this directory whose lifetime has ended.
function(gpenmpc_apply_source_overlays)
  include("${EXTERNAL_MODULES_LOCATION}/../PreparedPaths.cmake")
  if(NOT TARGET modules__mavlink OR NOT TARGET drivers__pwm_out)
    message(FATAL_ERROR "The expected FMUv6C MAVLink/PWM targets are missing")
  endif()
  # This exact parent declares UNITY_BUILD: the main target contains one
  # generated cat file, while _original retains the 19 actual source entries.
  # Replace that complete source set and derive a private unity include entry.
  if(NOT TARGET modules__mavlink_original)
    message(FATAL_ERROR "Expected parent MAVLink UNITY_BUILD source target")
  endif()
  get_target_property(_sources modules__mavlink_original SOURCES)
  set(_new_sources)
  set(_unity_content "// MAVLink sources in target order.\n")
  set(_changed 0)
  foreach(_source IN LISTS _sources)
    get_filename_component(_name "${_source}" NAME)
    if(EXISTS "${GPENMPC_MAVLINK_OVERLAY}/${_name}")
      list(APPEND _new_sources "${GPENMPC_MAVLINK_OVERLAY}/${_name}")
      string(APPEND _unity_content "#include \"${GPENMPC_MAVLINK_OVERLAY}/${_name}\"\n")
      math(EXPR _changed "${_changed}+1")
    else()
      message(FATAL_ERROR "Uncovered actual MAVLink TU: ${_source}")
    endif()
  endforeach()
  if(NOT _changed EQUAL 19)
    message(FATAL_ERROR "MAVLink source closure unexpectedly incomplete")
  endif()
  set_property(TARGET modules__mavlink_original PROPERTY SOURCES "${_new_sources}")
  set(_private_unity "${PX4_BINARY_DIR}/external_modules/gpenmpc_mavlink_unity.cpp")
  # Literal includes only, no generator expressions: materialize at configure
  # so the parent pre-3.20 source-property scope cannot misclassify it missing.
  if(EXISTS "${_private_unity}")
    file(READ "${_private_unity}" _previous_unity_content)
  endif()
  if(NOT _previous_unity_content STREQUAL _unity_content)
    file(WRITE "${_private_unity}" "${_unity_content}")
  endif()
  set_source_files_properties("${_private_unity}"
    TARGET_DIRECTORY modules__mavlink PROPERTIES GENERATED TRUE)
  set_property(TARGET modules__mavlink PROPERTY SOURCES "${_private_unity}")
  foreach(_mavlink_target modules__mavlink_original modules__mavlink)
    # Snapshot/feedback stream headers expose the real core ABI. Both parent
    # source targets must consume the same generated headers and NuttX shim.
    target_include_directories(${_mavlink_target} BEFORE PRIVATE
      "${GPENMPC_MAVLINK_OVERLAY}" "${GPENMPC_RFLY}/px4_stream"
      "${GPENMPC_RFLY}/local_source_ingress"
      "${GPENMPC_INGRESS}" "${GPENMPC_GENERATED}"
      "${GPENMPC_RFLY}/official_io_nuttx/overlay")
    target_include_directories(${_mavlink_target} SYSTEM PRIVATE
      "${GPENMPC_INGRESS}/nuttx_include_overlay")
  endforeach()
  target_include_directories(gpenmpc_rfly_runtime_components PRIVATE
    "${PX4_SOURCE_DIR}/src/modules/mavlink")
  target_link_libraries(gpenmpc_rfly_runtime_components PRIVATE mavlink_c)
  # mavlink_c exports generated dialect headers as INTERFACE_SOURCES. Their
  # parent's old GENERATED property is directory-scoped; propagate the actual
  # generation fact to these external consuming directories, not empty files.
  get_target_property(_dialect_headers mavlink_c INTERFACE_SOURCES)
  set_source_files_properties(${_dialect_headers}
    TARGET_DIRECTORY gpenmpc_rfly_runtime_components modules__gpenmpc_rfly_canonical modules__gpenmpc_rfly_session
    PROPERTIES GENERATED TRUE)
  add_dependencies(gpenmpc_rfly_runtime_components mavlink_c_generate)
  # Link after platforms/common creates the target.
  target_link_libraries(modules__gpenmpc_rfly_canonical PRIVATE px4_platform)
  target_link_libraries(modules__gpenmpc_rfly_session PRIVATE px4_platform mavlink_c)
  # The actual stream and eventual module use one linked registry object.
  # Do not instantiate private registry storage in individual translation units.
  target_link_libraries(modules__mavlink PRIVATE gpenmpc_rfly_runtime_components)
  # Match ordinary PX4 libraries: private NuttX context/config headers and
  # platform-generated includes must be ready, not inherited stale copies.
  add_dependencies(gpenmpc_rfly_runtime_components prebuild_targets uorb_headers)
  add_dependencies(gpenmpc_full_inner_facade prebuild_targets uorb_headers)
  add_dependencies(gpenmpc_full_inner_private prebuild_targets uorb_headers)
  # PX4 flattens module archives into its executable link; an INTERFACE
  # option on the facade is not propagated by that collection. Bind the
  # heading-reference hook to the actual final executable, not just a library.
  target_link_options(px4 PRIVATE "LINKER:--wrap=gpenmpcDesiredSe3Command")
  target_link_options(px4 PRIVATE "LINKER:--wrap=se3WrenchKernel")

  # ReadOnlyPwmState must compile in the actual driver's parameter/MODULE_NAME
  # context. Never instantiate a second private ModuleBase task slot.
  target_sources(drivers__pwm_out PRIVATE "${GPENMPC_RUNTIME}/ReadOnlyPwmState.cpp")
  # This helper is a separate archive member. The original px4 link line visits
  # drivers__pwm_out before the component consumer and cannot extract it then.
  # Express the actual read-only dependency so CMake emits it after the consumer.
  target_link_libraries(gpenmpc_rfly_runtime_components PRIVATE drivers__pwm_out)
  foreach(_pair "drivers__px4io|px4io.cpp|px4io_readonly.cpp"
                "drivers__dshot|DShot.cpp|DShot_readonly.cpp")
    string(REPLACE "|" ";" _parts "${_pair}")
    list(GET _parts 0 _target)
    list(GET _parts 1 _original)
    list(GET _parts 2 _replacement)
    if(NOT TARGET ${_target})
      message(FATAL_ERROR "Missing driver target ${_target}")
    endif()
    get_target_property(_sources ${_target} SOURCES)
    set(_next)
    set(_count 0)
    foreach(_source IN LISTS _sources)
      get_filename_component(_name "${_source}" NAME)
      if(_name STREQUAL _original)
        list(APPEND _next "${GPENMPC_DRIVER_OVERLAY}/${_replacement}")
        math(EXPR _count "${_count}+1")
      else()
        list(APPEND _next "${_source}")
      endif()
    endforeach()
    if(NOT _count EQUAL 1)
      message(FATAL_ERROR "Expected exactly one driver source ${_original}")
    endif()
    set_property(TARGET ${_target} PROPERTY SOURCES "${_next}")
  endforeach()
  target_include_directories(drivers__px4io PRIVATE "${PX4_SOURCE_DIR}/src/drivers/px4io")
  target_include_directories(drivers__dshot PRIVATE "${PX4_SOURCE_DIR}/src/drivers/dshot")
  # Preserve the original legacy executive except its single writer-admission
  # check. ModuleBase serializes task_spawn with the actual module mutex.
  set(_legacy_target modules__gpenmpc_trajectory_exec)
  if(NOT TARGET ${_legacy_target})
    # Exclude the legacy executor and retain its parameter metadata.
    target_compile_definitions(gpenmpc_rfly_runtime_components PRIVATE GPENMPC_LEGACY_EXECUTOR_NOT_LINKED=1)
    message(STATUS "GPENMPC parameter-only compatibility configured")
  else()
    message(FATAL_ERROR "Conflicting GPENMPC trajectory executable")
  endif()
  if(TARGET ${_legacy_target})
  get_target_property(_legacy_sources ${_legacy_target} SOURCES)
  set(_legacy_next)
  set(_legacy_count 0)
  foreach(_source IN LISTS _legacy_sources)
    get_filename_component(_name "${_source}" NAME)
    if(_name STREQUAL "GPENMPCTrajectoryExec.cpp")
      list(APPEND _legacy_next "${EXTERNAL_MODULES_LOCATION}/../overlay/src/modules/gpenmpc_trajectory_exec/GPENMPCTrajectoryExec.cpp")
      math(EXPR _legacy_count "${_legacy_count}+1")
    else()
      list(APPEND _legacy_next "${_source}")
    endif()
  endforeach()
  if(NOT _legacy_count EQUAL 1)
    message(FATAL_ERROR "Expected exact legacy executive source")
  endif()
  set_property(TARGET ${_legacy_target} PROPERTY SOURCES "${_legacy_next}")
  target_include_directories(${_legacy_target} PRIVATE
    "${PX4_SOURCE_DIR}/src/modules/gpenmpc_trajectory_exec"
    "${GPENMPC_RFLY}/px4_stream")
  target_link_libraries(${_legacy_target} PRIVATE gpenmpc_rfly_runtime_components)
  endif()
  message(STATUS "GPENMPC session modules configured")
endfunction()
cmake_language(DEFER DIRECTORY "${CMAKE_SOURCE_DIR}" CALL gpenmpc_apply_source_overlays)
