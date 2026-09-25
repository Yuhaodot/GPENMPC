#include "GPENMPCRflyHilStream.hpp"

// Force actual production factory code generation. No public authority-injecting
// bypass exists; an unbound shared registry is unavailable.
MavlinkStream *gpenmpc_compile_default_stream(Mavlink *mavlink)
{
    return MavlinkStreamGPENMPCRflyHILActuatorControls::new_instance(mavlink);
}
