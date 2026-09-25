#pragma once
// Extern uORB metadata declarations for vendor topic aliases.
// Include PX4 v1.16 actuator structures; alias metadata must be linked.
#include <uORB/uORB.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_outputs.h>
ORB_DECLARE(actuator_armed_rfly);
ORB_DECLARE(actuator_outputs_rfly);
