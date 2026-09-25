// Appended only to the private derived mavlink_main.cpp after its original
// definitions. Keep this callback short; stream configuration is performed by
// the existing separate `mavlink stream` command, never while this lock is held.
using GPENMPCLockedMavlinkVisitor=bool (*)(Mavlink &,void *);
bool gpenmpc_visit_locked_mavlink_device(const char *device,GPENMPCLockedMavlinkVisitor visitor,void *state) noexcept
{
    if(!device||!visitor)return false;
    LockGuard guard{mavlink_module_mutex};
    for(Mavlink *instance:mavlink_module_instances){
        if(instance&&instance->get_protocol()==Protocol::SERIAL&&instance->_device_name&&std::strcmp(instance->_device_name,device)==0)
            return visitor(*instance,state);
    }
    return false;
}
