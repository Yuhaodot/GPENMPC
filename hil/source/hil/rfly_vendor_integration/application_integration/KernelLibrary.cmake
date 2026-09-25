set(GPENMPC_KERNEL_DIR "${GPENMPC_HIL_SOURCE_DIR}/evidence/tracking_controller/generated")
file(GLOB GPENMPC_KERNEL_SOURCES CONFIGURE_DEPENDS "${GPENMPC_KERNEL_DIR}/*.c")
list(LENGTH GPENMPC_KERNEL_SOURCES _kernel_count)
if(NOT _kernel_count EQUAL 74)
  message(FATAL_ERROR "Expected 74 controller C translation units")
endif()
list(REMOVE_ITEM GPENMPC_KERNEL_SOURCES "${GPENMPC_KERNEL_DIR}/gpenmpcObserveCausalVerticalDisturbance.c")
list(APPEND GPENMPC_KERNEL_SOURCES
  "${GPENMPC_HIL_SOURCE_DIR}/live/vertical_observer/gpenmpcObserveCausalVerticalDisturbance.c")
add_library(gpenmpc_full_inner_private STATIC EXCLUDE_FROM_ALL ${GPENMPC_KERNEL_SOURCES})
target_include_directories(gpenmpc_full_inner_private PRIVATE "${GPENMPC_KERNEL_DIR}")
target_include_directories(gpenmpc_full_inner_private SYSTEM PRIVATE "${GPENMPC_MATLAB_INCLUDE}")
target_compile_options(gpenmpc_full_inner_private PRIVATE
  -ffp-contract=off -Wno-address-of-packed-member -Wno-cast-align -Wno-error=float-equal
  -fstack-usage "SHELL:-include \"${GPENMPC_RFLY}/CanonicalCombinedSymbolNamespace.h\"")
target_compile_definitions(gpenmpc_full_inner_private PRIVATE MODULE_NAME="gpenmpc_full_inner")
set(GPENMPC_IDENTITY_DIR "${PX4_BINARY_DIR}/gpenmpc_identity")
file(GLOB _identity_generated CONFIGURE_DEPENDS "${GPENMPC_KERNEL_DIR}/*.c" "${GPENMPC_KERNEL_DIR}/*.h")
set(_identity_facade
  "${GPENMPC_RFLY}/full_inner_abi/CanonicalFullInnerAbi.cpp"
  "${GPENMPC_RFLY}/full_inner_abi/CanonicalFullInnerAbi.h"
  "${GPENMPC_RFLY}/CanonicalCombinedSymbolNamespace.h"
  "${GPENMPC_RFLY}/CanonicalLocalInnerStateStore.hpp"
  "${GPENMPC_RFLY}/CanonicalReferenceStateStore.hpp"
  "${GPENMPC_RFLY}/CanonicalJointStateInstaller.hpp"
  "${GPENMPC_RFLY}/CanonicalOperatorReference.hpp"
  "${GPENMPC_HIL_SOURCE_DIR}/px4_full_inner/consumption/CanonicalSha256.hpp"
  "${GPENMPC_HIL_SOURCE_DIR}/px4_full_inner/portable/CanonicalPortable.hpp")
add_custom_command(OUTPUT "${GPENMPC_IDENTITY_DIR}/FullInnerBuildIdentity.h"
  "${GPENMPC_IDENTITY_DIR}/source_identity.json"
  COMMAND "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/write_build_identity.py"
    --hil-source "${GPENMPC_HIL_SOURCE_DIR}" --archive "$<TARGET_FILE:gpenmpc_full_inner_private>"
    --output "${GPENMPC_IDENTITY_DIR}"
  DEPENDS gpenmpc_full_inner_private ${_identity_generated} ${_identity_facade}
    "${CMAKE_CURRENT_LIST_DIR}/write_build_identity.py"
  VERBATIM)
add_custom_target(gpenmpc_build_identity DEPENDS "${GPENMPC_IDENTITY_DIR}/FullInnerBuildIdentity.h")
