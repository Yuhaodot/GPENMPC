#include "../px4_wire/CanonicalLocalSnapshotWire.hpp"
namespace sw=gpenmpc_local_snapshot_wire;
extern "C" {
bool gpenmpc_probe_snapshot_from_actual(const gpenmpc_odometry::Snapshot*s,
 const gpenmpc_hil_endpoint_reader::Receipt*r,sw::Bytes*b){return sw::encode_from_actual(*s,*r,*b);}
bool gpenmpc_probe_snapshot_decode(const sw::Bytes*b,sw::Observation*out){return sw::decode(*b,*out);}
bool gpenmpc_probe_snapshot_fragment(const sw::Bytes*b,unsigned i,sw::Fragment*out){return sw::fragment(*b,i,*out);}
unsigned char gpenmpc_size_snapshot_observation[sizeof(sw::Observation)];
unsigned char gpenmpc_size_original_receipt[sizeof(gpenmpc_hil_endpoint_reader::Receipt)];
}
