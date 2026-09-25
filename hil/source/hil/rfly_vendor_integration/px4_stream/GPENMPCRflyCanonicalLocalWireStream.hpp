#ifndef GPENMPC_RFLY_CANONICAL_LOCAL_WIRE_STREAM_HPP
#define GPENMPC_RFLY_CANONICAL_LOCAL_WIRE_STREAM_HPP
#include "mavlink_main.h"
#include "mavlink_stream.h"
#include "CanonicalLocalWireRouteRegistry.hpp"

// Actual existing MAVLink task/channel/transport. No socket, second owner,
// arm/mode command, scheduler, old Pump or numerical execution in this class.
class MavlinkStreamGPENMPCRflyCanonicalLocalWire final:public MavlinkStream {
public:
    static MavlinkStream*new_instance(Mavlink*m){return new MavlinkStreamGPENMPCRflyCanonicalLocalWire(m);}
    static constexpr const char*get_name_static(){return "GPENMPC_LOCAL_WIRE";}
    static constexpr uint16_t get_id_static(){return MAVLINK_MSG_ID_TUNNEL;}
    const char*get_name()const override{return get_name_static();}
    uint16_t get_id()override{return get_id_static();}
    unsigned get_size()override{return MAVLINK_MSG_ID_TUNNEL_LEN+MAVLINK_NUM_NON_PAYLOAD_BYTES;}
    unsigned get_size_avg()override{return gpenmpc_rfly_px4::CanonicalLocalWireOutbox::max_fragments*get_size();}
    ~MavlinkStreamGPENMPCRflyCanonicalLocalWire()override{
        if(registry_)(void)registry_->release_view(link_,this);
        // The route registry owns no outbox memory. Context keeps it alive
        // until quiescent unbind; view retirement closes any original tail.
    }
    struct Diagnostics {
        std::uint64_t copied_fragments{},send_attempts{},send_returns{},completed_copy_batches{},
            original_last_send_call_us{},original_last_send_return_us{};
        bool failed{},expired{},clock_regressed{};
    };
    // Read from the actual MAVLink task or AFTER task/callback quiescence only.
    // These uint64 diagnostics are not advertised as cross-task atomic reads.
    const Diagnostics&quiescent_diagnostics()const noexcept{return diagnostics_;}
    const gpenmpc_rfly_px4::LocalWireFragment&quiescent_retained_fragment()const noexcept{return fragment_;}
private:
    explicit MavlinkStreamGPENMPCRflyCanonicalLocalWire(Mavlink*m):MavlinkStream(m){}
    bool time_valid(std::uint64_t now)noexcept{
        const auto&m=fragment_.original;
        if(!m.original_anchor_us||!m.original_transport_valid_until_us||now<m.original_anchor_us){
            diagnostics_.clock_regressed=true;return false;
        }
        // Same component-only distinction as the actual outbox. No wire
        // timestamp or expiry is rewritten; this is delayed history, not a
        // fresh state/GP result. Full-method and all numerical traffic retain
        // their original transport deadline, including the post-send check.
        if(now>m.original_transport_valid_until_us&&
           !(m.component_history_only&&m.schema==gpenmpc_local_committed_wire::schema)){
            diagnostics_.expired=true;return false;}
        return true;
    }
    void stop_view()noexcept{
        diagnostics_.failed=true;
        if(registry_)(void)registry_->release_view(link_,this);
        // Never reset, relabel, refresh a deadline, or claim unsent bytes were
        // sent. Actual GP owner still owns its original pending request.
    }
    bool send()override{
        using namespace gpenmpc_rfly_stream;
        using gpenmpc_rfly_px4::LocalWireTake;
        if(diagnostics_.failed)return false;
        if(!resolved_){
            auto*l=link_lifetime_registry();
            if(!l||l->lookup(_mavlink,link_)!=LinkAccess::Read)return false;
            resolved_=true; // same-address replacement cannot renew this token
        }
        if(!registry_)registry_=canonical_local_wire_route_registry();
        if(!registry_)return false;
        bool attempted=false;
        for(unsigned count=0;count<gpenmpc_rfly_px4::CanonicalLocalWireOutbox::max_fragments;++count){
            // Capacity BEFORE consuming. A previous short buffer merely pauses
            // the same immutable message; it never advances an unsent index.
            if(_mavlink->get_free_tx_buf()<get_size())break;
            if(diagnostics_.copied_fragments==UINT64_MAX||diagnostics_.send_attempts==UINT64_MAX||
               diagnostics_.send_returns==UINT64_MAX||diagnostics_.completed_copy_batches==UINT64_MAX){stop_view();return attempted;}
            const auto taken=registry_->take(link_,this,fragment_,0);
            if(taken==LocalWireTake::Expired){diagnostics_.expired=true;stop_view();return attempted;}
            if(taken!=LocalWireTake::Fragment)break;
            ++diagnostics_.copied_fragments;
            mavlink_tunnel_t message{};
            message.target_system=fragment_.target_system;message.target_component=fragment_.target_component;
            message.payload_type=42002;message.payload_length=fragment_.fragment.length;
            std::memcpy(message.payload,fragment_.fragment.payload,fragment_.fragment.length);
            // Check after message preparation, immediately before entering the
            // real sender. Any late RETURN still records the occurred call.
            const auto before=hrt_absolute_time();
            if(!time_valid(before)){stop_view();return attempted;}
            diagnostics_.original_last_send_call_us=before;++diagnostics_.send_attempts;
            // Real v1.16 convenience sender on THIS original channel. It is a
            // void API: returned != delivered, and never equals a GP reply.
            mavlink_msg_tunnel_send_struct(_mavlink->get_channel(),&message);
            ++diagnostics_.send_returns;attempted=true;
            const auto after=hrt_absolute_time();diagnostics_.original_last_send_return_us=after;
            if(after<before||!time_valid(after)){
                if(after<before)diagnostics_.clock_regressed=true;
                stop_view();return attempted; // already attempted prefix retained
            }
            const unsigned original_index=fragment_.fragment.payload[0]&0x0f;
            if(original_index+1==fragment_.original.fragment_count){
                ++diagnostics_.completed_copy_batches;break;
            }
            // Component history yields after each fragment; full-method records yield
            // after a bounded RLS-sized slice. Each take prioritizes the numerical lane
            // and validates TX capacity, fragment index, checksum and expiry.
            if(fragment_.original.yield_after_fragment)break;
        }
        return attempted; // scheduling result only, NO transport success ACK
    }
    gpenmpc_rfly_stream::CanonicalLocalWireRouteRegistry*registry_{};
    gpenmpc_rfly_stream::LinkToken link_{};bool resolved_{};
    gpenmpc_rfly_px4::LocalWireFragment fragment_{};
    Diagnostics diagnostics_{};
};
#endif
