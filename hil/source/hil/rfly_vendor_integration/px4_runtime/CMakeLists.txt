# Runtime components used by application_integration.
px4_add_module(
    MODULE modules__gpenmpc_rfly_canonical
    MAIN gpenmpc_rfly_canonical
    SRCS
        GPENMPCRflyCanonicalModule.cpp
        Px4CanonicalIo.cpp
    INCLUDES
        ../../evidence/arm_controller/GPENMPC_Rfly_Canonical_Controller_ert_rtw
    COMPILE_FLAGS
        -ffp-contract=off
    DEPENDS
        px4_platform
)
