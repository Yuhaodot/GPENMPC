#pragma once
// Receive-only transport conversion. Input is a frame admitted by PX4's real
// MAVLink parser. This helper does not authenticate, reassemble kernel ABI,
// assign estimator identity, publish motors, arm, or confer execution authority.
#include <mavlink.h>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace gpenmpc_ingress {
constexpr std::uint16_t payload_type = 42002; // local experiment; legacy RTA1 remains 42001
constexpr std::size_t payload_capacity = 128;
constexpr std::size_t queue_capacity = 8;
static_assert(payload_type >= 32768 && payload_type != 42001, "Independent experimental channel");
static_assert(MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN == payload_capacity, "MAVLink schema mismatch");

enum class Result : std::uint8_t {
    Accepted, NotTunnel, OtherPayload, BadVersion, BadWireLength, BadPayloadLength,
    BadLocalRoute, BadSource, WrongTarget, BadTimestamp, SequenceExhausted, QueueFull, PublicationFailure, Count
};
struct Fields {
    std::uint64_t timestamp{}, reception_sequence{};
    std::uint8_t receiver_instance{}, source_system{}, source_component{}, target_system{}, target_component{};
    std::uint8_t mavlink_sequence{}, wire_payload_length{};
    std::uint16_t payload_type{};
    std::uint8_t payload_length{};
    std::array<std::uint8_t, payload_capacity> payload{};
};
struct Counters {
    std::array<std::uint64_t, static_cast<std::size_t>(Result::Count)> outcomes{};
    std::uint64_t publication_failures{}, published{};
    Result last_result{Result::Accepted};
    std::uint64_t last_event_hrt{}, last_rejected_hrt{};
    Result first_fault{Result::Accepted},last_fault{Result::Accepted};
    std::uint64_t first_fault_hrt{},last_fault_hrt{},rejected_total{};
};

inline bool belongs_to_channel(const mavlink_message_t &frame) noexcept {
    return frame.msgid==MAVLINK_MSG_ID_TUNNEL && frame.len>=2
        && mavlink_msg_tunnel_get_payload_type(&frame)==payload_type;
}

template<class Topic> void copy_to_topic(const Fields &in, const Counters &counters, Topic &out) noexcept {
    out = {};
    out.timestamp=in.timestamp;out.reception_sequence=in.reception_sequence;
    out.receiver_instance=in.receiver_instance;out.source_system=in.source_system;out.source_component=in.source_component;
    out.target_system=in.target_system;out.target_component=in.target_component;
    out.mavlink_sequence=in.mavlink_sequence;out.wire_payload_length=in.wire_payload_length;
    out.payload_type=in.payload_type;out.payload_length=in.payload_length;
    for(std::size_t i=0;i<payload_capacity;++i) out.payload[i]=in.payload[i];
    out.ingress_first_fault=static_cast<std::uint8_t>(counters.first_fault);
    out.ingress_first_fault_hrt=counters.first_fault_hrt;
    out.ingress_last_fault=static_cast<std::uint8_t>(counters.last_fault);
    out.ingress_last_fault_hrt=counters.last_fault_hrt;out.ingress_rejected_total=counters.rejected_total;
    out.ingress_queue_overflows=counters.outcomes[static_cast<std::size_t>(Result::QueueFull)];
    out.ingress_publication_failures=counters.publication_failures;
}

class Receiver {
public:
    Result receive(const mavlink_message_t &frame, std::uint64_t ingress_hrt_us,
                   std::uint8_t local_system, std::uint8_t local_component,
                   std::uint8_t receiver_instance) noexcept {
        if(frame.msgid!=MAVLINK_MSG_ID_TUNNEL) return record(Result::NotTunnel,ingress_hrt_us);
        // Preserve every other TUNNEL protocol's previous routing semantics.
        if(frame.len<2) return record(Result::BadWireLength,ingress_hrt_us);
        if(mavlink_msg_tunnel_get_payload_type(&frame)!=payload_type)
            return record(Result::OtherPayload,ingress_hrt_us);
        if(frame.magic!=MAVLINK_STX) return record(Result::BadVersion,ingress_hrt_us);
        // MAVLink 2 legitimately trims trailing zero bytes. Decode must restore
        // those bytes; requiring the nominal 133-byte length would reject them.
        if(frame.len<5 || frame.len>MAVLINK_MSG_ID_TUNNEL_LEN) return record(Result::BadWireLength,ingress_hrt_us);
        mavlink_tunnel_t tunnel{};
        mavlink_msg_tunnel_decode(&frame,&tunnel);
        if(tunnel.payload_length==0 || tunnel.payload_length>payload_capacity)
            return record(Result::BadPayloadLength,ingress_hrt_us);
        if(!local_system || !local_component) return record(Result::BadLocalRoute,ingress_hrt_us);
        if(!frame.sysid || !frame.compid) return record(Result::BadSource,ingress_hrt_us);
        // New channel requires an exact unicast target. No change to legacy/broadcast paths.
        if(tunnel.target_system!=local_system || tunnel.target_component!=local_component)
            return record(Result::WrongTarget,ingress_hrt_us);
        if(!ingress_hrt_us) return record(Result::BadTimestamp,ingress_hrt_us);
        if(sequence_==std::numeric_limits<std::uint64_t>::max()) return record(Result::SequenceExhausted,ingress_hrt_us);
        ++sequence_; // queue-full losses remain visible to the next downstream publication
        if(size_==queue_capacity) return record(Result::QueueFull,ingress_hrt_us);
        Fields &out=queue_[(head_+size_)%queue_capacity];out={};
        out.timestamp=ingress_hrt_us;out.reception_sequence=sequence_;out.receiver_instance=receiver_instance;
        out.source_system=frame.sysid;out.source_component=frame.compid;
        out.target_system=tunnel.target_system;out.target_component=tunnel.target_component;
        out.mavlink_sequence=frame.seq;out.wire_payload_length=frame.len;
        out.payload_type=tunnel.payload_type;out.payload_length=tunnel.payload_length;
        for(std::size_t i=0;i<tunnel.payload_length;++i)out.payload[i]=tunnel.payload[i];
        ++size_;
        return record(Result::Accepted,ingress_hrt_us);
    }
    // Call only after actual publication succeeds. The patch uses the real
    // uORB Publication::publish result, never a caller safety/authority boolean.
    template<class Publisher> bool drain(std::uint64_t attempt_hrt,Publisher &&publisher) noexcept {
        while(size_) {
            if(!publisher(queue_[head_])){++counters_.publication_failures;record(Result::PublicationFailure,attempt_hrt);return false;}
            queue_[head_]={};head_=(head_+1)%queue_capacity;--size_;++counters_.published;
        }
        return true;
    }
    const Counters &counters() const noexcept{return counters_;}
    std::size_t pending() const noexcept{return size_;}
    const Fields *front() const noexcept{return size_ ? &queue_[head_] : nullptr;}
private:
    Result record(Result result,std::uint64_t hrt) noexcept {
        ++counters_.outcomes[static_cast<std::size_t>(result)];counters_.last_result=result;counters_.last_event_hrt=hrt;
        if(result!=Result::Accepted && result!=Result::NotTunnel && result!=Result::OtherPayload){
            if(counters_.first_fault==Result::Accepted){counters_.first_fault=result;counters_.first_fault_hrt=hrt;}
            counters_.last_fault=result;counters_.last_fault_hrt=hrt;
            if(result!=Result::PublicationFailure){counters_.last_rejected_hrt=hrt;++counters_.rejected_total;}
        }
        return result;
    }
    std::array<Fields,queue_capacity> queue_{};
    std::size_t head_{},size_{};std::uint64_t sequence_{};Counters counters_{};
};
} // namespace gpenmpc_ingress
