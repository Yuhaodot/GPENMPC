#include "GPENMPCRflyCommittedFeedbackStream.hpp"
extern "C" MavlinkStream*gpenmpc_compile_committed_feedback_factory(Mavlink*m){return MavlinkStreamGPENMPCRflyCommittedFeedback::new_instance(m);}
