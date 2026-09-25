#include "LinkLifetimeRegistry.hpp"
#include "../px4_runtime/ExecutionSessionRegistration.hpp"

namespace gpenmpc_rfly_stream {

CanonicalBind LinkLifetimeRegistry::bind_canonical(const gpenmpc_rfly_px4::SessionEcho &echo,
    std::uint8_t host_system,std::uint8_t host_component,const void *owner,CanonicalReservation &out)noexcept
{
    out={};const HostHeartbeatBinding binding{echo.process_session_generation,echo.board_registration_hrt_us,host_system,host_component};
    if(!owner||!echo.link.pointer||!echo.link.generation||!echo.observed_identity.uid||
       !echo.observed_identity.system||!echo.observed_identity.component||!valid(binding))return CanonicalBind::Invalid;
    if(!mutex_.lock())return CanonicalBind::Unproven;
    Entry *selected=nullptr;CanonicalBind result=CanonicalBind::Unavailable;
    // The global topic has one canonical reservation across every actual link.
    // Native handlers entered before this lock must leave before bind can pass.
    for(auto &entry:entries_){
        if(entry.reservation.generation||entry.native_borrow_generation){result=CanonicalBind::Busy;selected=nullptr;break;}
        if(entry.state==State::Live&&entry.token==echo.link)selected=&entry;
    }
    if(selected&&next_reservation_generation_!=UINT64_MAX&&selected->heartbeat.bind(binding)){
        selected->reservation={echo.link,owner,++next_reservation_generation_,binding};
        out=selected->reservation;result=CanonicalBind::Bound;
    }
    // On uncertain unlock the exact possible reservation remains available for
    // synchronous abort cleanup; never advertise Bound from that uncertainty.
    return mutex_.unlock()?result:CanonicalBind::Unproven;
}

LinkAccess LinkLifetimeRegistry::release_canonical(const CanonicalReservation &reservation)noexcept
{
    if(!reservation.generation||!reservation.owner)return LinkAccess::Rejected;
    if(!mutex_.lock())return LinkAccess::Unproven;
    LinkAccess result=LinkAccess::Rejected;
    for(auto &entry:entries_)if(entry.reservation==reservation){
        entry.heartbeat.retire();entry.reservation={};
        if(entry.state==State::Closing&&!entry.native_borrow_generation)entry={};
        result=LinkAccess::Read;break;
    }
    return mutex_.unlock()?result:LinkAccess::Unproven;
}

LinkAccess LinkLifetimeRegistry::read_heartbeat(const CanonicalReservation &reservation,
    std::uint64_t now,std::uint64_t max_age,HostHeartbeatSnapshot &out)noexcept
{
    out={};if(!reservation.generation||!reservation.owner)return LinkAccess::Unavailable;
    if(!mutex_.lock())return LinkAccess::Unproven;
    LinkAccess result=LinkAccess::Unavailable;
    for(const auto &entry:entries_)if(entry.state==State::Live&&entry.reservation==reservation){
        out=entry.heartbeat.snapshot(now,max_age);result=LinkAccess::Read;break;
    }
    if(!mutex_.unlock()){out={};return LinkAccess::Unproven;}return result;
}

HeartbeatRecord LinkLifetimeRegistry::record_heartbeat(const void *actual_link,
    std::uint8_t system,std::uint8_t component,std::uint8_t type,std::uint8_t autopilot,
    std::uint64_t original_receiver_hrt_us)noexcept
{
    if(!actual_link||!mutex_.lock())return HeartbeatRecord::Unbound;
    HeartbeatRecord result=HeartbeatRecord::Unbound;
    for(auto &entry:entries_)if(entry.state==State::Live&&entry.token.pointer==actual_link&&entry.reservation.generation){
        result=entry.heartbeat.record(system,component,type,autopilot,original_receiver_hrt_us);break;
    }
    return mutex_.unlock()?result:HeartbeatRecord::Unbound;
}

LinkAccess LinkLifetimeRegistry::native_offboard_allowed()noexcept
{
    if(!mutex_.lock())return LinkAccess::Unproven;
    LinkAccess result=LinkAccess::Read;
    for(const auto &entry:entries_)if(entry.reservation.generation){result=LinkAccess::Rejected;break;}
    return mutex_.unlock()?result:LinkAccess::Unproven;
}

LinkAccess LinkLifetimeRegistry::enter_native_offboard(const void *actual_link,NativeBorrowToken &out)noexcept
{
    out={};if(!actual_link)return LinkAccess::Unavailable;
    if(!mutex_.lock())return LinkAccess::Unproven;
    Entry *selected=nullptr;LinkAccess result=LinkAccess::Unavailable;
    for(auto &entry:entries_){
        if(entry.reservation.generation){result=LinkAccess::Rejected;selected=nullptr;break;}
        if(entry.state==State::Live&&entry.token.pointer==actual_link)selected=&entry;
    }
    if(selected){
        // A real receiver handles these messages serially; a second entered
        // scope on the same link is rejected, not collapsed into a boolean.
        result=LinkAccess::Rejected;
        if(!selected->native_borrow_generation&&next_native_borrow_generation_!=UINT64_MAX){
            selected->native_borrow_generation=++next_native_borrow_generation_;
            out={selected->token,selected->native_borrow_generation};result=LinkAccess::Read;
        }
    }
    return mutex_.unlock()?result:LinkAccess::Unproven;
}

LinkAccess LinkLifetimeRegistry::leave_native_offboard(const NativeBorrowToken &borrow)noexcept
{
    if(!borrow.generation||!borrow.link.pointer||!borrow.link.generation)return LinkAccess::Rejected;
    if(!mutex_.lock())return LinkAccess::Unproven;
    LinkAccess result=LinkAccess::Rejected;
    for(auto &entry:entries_)if(entry.state!=State::Empty&&entry.token==borrow.link&&entry.native_borrow_generation==borrow.generation){
        entry.native_borrow_generation=0;
        if(entry.state==State::Closing&&!entry.reservation.generation)entry={};
        result=LinkAccess::Read;break;
    }
    return mutex_.unlock()?result:LinkAccess::Unproven;
}

} // namespace gpenmpc_rfly_stream
