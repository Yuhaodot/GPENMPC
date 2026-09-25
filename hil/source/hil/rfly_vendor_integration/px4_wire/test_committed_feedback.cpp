#define main command_transport_fixture_main
#include "../test_slim_argument_transport.cpp"
#undef main
#include "CommittedFeedbackWire.hpp"
namespace fw=gpenmpc_feedback_wire;
namespace px=gpenmpc_rfly_px4;
int main(int argc,char**argv){try{if(argc!=3)return 2;const auto fixtures=read_kernel(argv[1]);const auto states=read_state(argv[2]);
    std::ofstream saved("ACTUAL_COMMITTED_FEEDBACK_PACKETS.bin",std::ios::binary);saved.write("RFC5",4);const std::uint8_t denominator[4]={0,0,0,76};saved.write(reinterpret_cast<const char*>(denominator),4);
    unsigned frames=0,total_wire=0,min_wire=UINT32_MAX,max_wire=0;
    for(unsigned row=0;row<76;++row){const auto&f=fixtures[row];std::uint64_t now=1000900;const auto config=execution_config(f);
        gpenmpc_rfly_slim::SnapshotExecutor<>store(source_config(),config,{clock_now,&now});SnapshotTicket ticket{};
        check(store.capture(raw(states[row]),std::uint32_t(states[row].generation),1000300,id(),&topic,0,ticket),"actual private feedback source");
        const auto command=slim_command(f,ticket);NumericalPrepared prepared{};NumericalReceipt receipt{};
        check(store.prepareNumericalOnly(command,prepared)&&store.commitNumericalReceipt(ticket,ack(prepared,now),receipt),"actual step + explicit MOCK independent latch test receipt");
        px::CommittedFeedbackLatch latch;check(latch.record_success(prepared,receipt,now,1004000),"record actual61 same prepared/committed transaction");
        px::CommittedFeedback feedback{};check(latch.take(feedback,1001000)==px::FeedbackDisposition::Fresh,"fresh one-shot feedback");
        check(std::memcmp(feedback.actual61.data(),prepared.execution.actual61.data(),488)==0&&std::memcmp(feedback.published_control16.data(),prepared.execution.rfly_controls16.data(),64)==0,"all61/16 bits copied from actual step/receipt");
        px::CommittedFeedback duplicate{};check(latch.take(duplicate,1001000)==px::FeedbackDisposition::Empty,"repeat read cannot repeat evidence");
        fw::Bytes bytes{};check(fw::encode(feedback,bytes),"RFC1 encode complete token+61+16");saved.write(reinterpret_cast<const char*>(bytes.data()),bytes.size());
        gpenmpc_portable::Array<std::uint8_t,488>oracle{};gpenmpc_snapshot_wire::Writer expected(oracle.data());for(double v:prepared.execution.actual61)expected.real(v);
        saved.write(reinterpret_cast<const char*>(oracle.data()),oracle.size());fw::Bytes reassembled{};unsigned offset=0,row_wire=0;
        for(unsigned index=0;index<fw::fragment_count;++index){old::Fragment part{};check(fw::fragment(bytes,index,part),"feedback schema5 fragment");
            mavlink_message_t message{},buffer{},parsed{};mavlink_status_t tx{},parser{},status{};tx.current_tx_seq=std::uint8_t(row*10+index);
            mavlink_msg_tunnel_pack_status(1,1,&tx,&message,42,191,ingress::payload_type,part.length,part.payload);std::uint8_t wire[MAVLINK_MAX_PACKET_LEN]{};
            const auto n=mavlink_msg_to_send_buffer(wire,&message);unsigned good=0;for(unsigned k=0;k<n;++k)good+=mavlink_frame_char_buffer(&buffer,&parser,wire[k],&parsed,&status)==MAVLINK_FRAMING_OK;
            mavlink_tunnel_t decoded{};mavlink_msg_tunnel_decode(&parsed,&decoded);
            check(good==1&&parsed.sysid==1&&parsed.compid==1&&decoded.target_system==42&&decoded.target_component==191&&decoded.payload[0]==0x50+index&&old::get64(decoded.payload+1)==feedback.token.output_generation,"actual schema5 generated pack/parser preserves direction/generation");
            const unsigned count=decoded.payload_length-9;std::memcpy(reassembled.data()+offset,decoded.payload+9,count);offset+=count;
            const std::uint8_t length[2]={std::uint8_t(n>>8),std::uint8_t(n)};saved.write(reinterpret_cast<const char*>(length),2);saved.write(reinterpret_cast<const char*>(wire),n);
            row_wire+=n;++frames;
        }
        fw::Bytes roundtrip{};px::CommittedFeedback decoded{};
        check(reassembled==bytes&&fw::decode(reassembled,decoded)&&fw::encode(decoded,roundtrip)&&roundtrip==bytes,"actual wire1112 and everyfield bit exact decode/reencode");
        total_wire+=row_wire;min_wire=std::min(min_wire,row_wire);max_wire=std::max(max_wire,row_wire);
        if(row==0){
            px::CommittedFeedbackLatch historical;check(historical.record_success(prepared,receipt,now,1004000)&&historical.take(decoded,1004001)==px::FeedbackDisposition::HistoricalExpired&&std::memcmp(decoded.actual61.data(),feedback.actual61.data(),488)==0,"expired once raw preserved not fresh or renewed");
            check(fw::encode(decoded,roundtrip)&&fw::decode(roundtrip,decoded)&&decoded.disposition==px::FeedbackDisposition::HistoricalExpired,"historical status actually carried on wire");
            px::CommittedFeedbackLatch revoked;check(revoked.record_success(prepared,receipt,now,1004000),"revocation baseline");revoked.revoke();
            check(revoked.take(decoded,1001000)==px::FeedbackDisposition::Revoked&&std::memcmp(decoded.actual61.data(),feedback.actual61.data(),488)==0,"revocation retains one audit raw without fresh qualification");
            check(fw::encode(decoded,roundtrip)&&fw::decode(roundtrip,decoded)&&decoded.disposition==px::FeedbackDisposition::Revoked,"revoked status actually carried on wire");
            px::CommittedFeedbackLatch full;check(full.record_success(prepared,receipt,now,1004000)&&!full.record_success(prepared,receipt,now,1004000)&&full.take(decoded,1001000)==px::FeedbackDisposition::Revoked,"single-slot fail closed without overwriting original raw");
            for(unsigned mutation=0;mutation<6;++mutation){auto bad=prepared;auto wrong=receipt;
                if(mutation==0)bad.execution.actual61[0]+=1;if(mutation==1)wrong.snapshot_ticket[0]^=1;
                if(mutation==2)wrong.execution.generated_arm_source_sha256[0]^=1;if(mutation==3)wrong.execution.published_control[0]+=0.1f;
                if(mutation==4)bad.execution.actual61[20]=std::numeric_limits<double>::quiet_NaN();
                px::CommittedFeedbackLatch negative;check(!negative.record_success(bad,wrong,mutation==5?1000000:now,1004000),"mismatched payload/ticket/code/output/nonfinite/original time rejected");}
            auto corrupt=bytes;corrupt[600]^=1;check(!fw::decode(corrupt,decoded),"altered wire actual61 integrity rejected");
        }
    }
    saved.close();std::ofstream report("COMMITTED_FEEDBACK_RESULT.json");report<<"{\"checks\":"<<checks<<",\"failed\":"<<failures<<",\"actual61_rows\":76,\"actual_simulink_steps\":76,\"header_test_commit_backend\":\"EXPLICIT_MOCK_PENDING_REAL_IO_EDIT_WINDOW\",\"message_bytes\":1112,\"frames_per_feedback\":10,\"actual_generated_frames\":"<<frames<<",\"actual_wire_bytes\":"<<total_wire<<",\"minimum_wire_bytes\":"<<min_wire<<",\"maximum_wire_bytes\":"<<max_wire<<",\"host_recomputation_used\":false,\"board_access\":0}\n";
    std::cout<<"checks="<<checks<<" failed="<<failures<<" frames="<<frames<<" wire="<<total_wire<<" min="<<min_wire<<" max="<<max_wire<<'\n';return failures?1:0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
