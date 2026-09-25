// Compile only: verify the same full generated metadata and target object size.
#include "Px4CanonicalIo.hpp"
static_assert(ORB_TOPICS_COUNT==311,"all headers must use the same full alias plus ingress table");
static_assert(ORB_ID::actuator_outputs_rfly!=ORB_ID::actuator_outputs_sim,"single publisher topics remain distinct");
extern "C" {char gpenmpc_px4_io_target_size[sizeof(gpenmpc_rfly_px4::Px4CanonicalIo)];}
