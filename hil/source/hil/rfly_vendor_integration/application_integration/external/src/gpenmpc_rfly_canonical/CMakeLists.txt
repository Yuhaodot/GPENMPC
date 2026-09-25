include("${CMAKE_CURRENT_LIST_DIR}/../../../PreparedPaths.cmake")

# ModuleBase CLI uses the application-supplied context.
px4_add_module(
  MODULE modules__gpenmpc_rfly_canonical
  MAIN gpenmpc_rfly_canonical_local
  SRCS "${GPENMPC_RUNTIME}/GPENMPCRflyCanonicalLocalModule.cpp"
  INCLUDES "${GPENMPC_GENERATED}"
    "${GPENMPC_RFLY}/official_io_nuttx/overlay" "${GPENMPC_INGRESS}"
  COMPILE_FLAGS -ffp-contract=off -fno-fast-math -fstack-usage -DGPENMPC_CANONICAL_CLOSED_EVIDENCE=1
  DEPENDS gpenmpc_rfly_runtime_components)
target_include_directories(modules__gpenmpc_rfly_canonical SYSTEM PRIVATE
  "${GPENMPC_INGRESS}/nuttx_include_overlay")
