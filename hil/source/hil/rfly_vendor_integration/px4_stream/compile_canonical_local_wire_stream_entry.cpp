#include "GPENMPCRflyCanonicalLocalWireStream.hpp"
MavlinkStream*gpenmpc_real_canonical_local_wire_factory_probe(Mavlink*link){
    return MavlinkStreamGPENMPCRflyCanonicalLocalWire::new_instance(link);
}
extern "C" const unsigned char gpenmpc_local_wire_stream_sizeof[sizeof(MavlinkStreamGPENMPCRflyCanonicalLocalWire)]={};
