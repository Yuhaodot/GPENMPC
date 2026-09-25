#include "GPENMPCRflySnapshotStream.hpp"
MavlinkStream *gpenmpc_real_snapshot_factory_probe(Mavlink *link){return MavlinkStreamGPENMPCRflySnapshot::new_instance(link);}
