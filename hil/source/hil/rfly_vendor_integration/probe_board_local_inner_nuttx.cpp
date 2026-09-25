// Compile-only explicit instantiation: no application entry or provider.
#include "BoardLocalInnerSchedule.hpp"
template class gpenmpc_local_schedule::BoardLocalInnerSchedule<2>;
extern "C" {
char gpenmpc_local_target_size_Schedule[sizeof(gpenmpc_local_schedule::BoardLocalInnerSchedule<2>)];
char gpenmpc_local_target_size_RotorLag[sizeof(gpenmpc_local_schedule::RotorLag)];
char gpenmpc_local_target_size_GpReply[sizeof(gpenmpc_local_schedule::GpReply)];
}
