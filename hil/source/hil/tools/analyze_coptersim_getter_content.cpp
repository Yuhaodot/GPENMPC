// Analyze retained getter bytes offline.
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <array>
#include <cfenv>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>
#include <mavlink/common/mavlink.h>
#include "../rfly_vendor_integration/clock_tap_overlay/DllSharedGetterTap.hpp"
#include "../rfly_vendor_integration/clock_tap_overlay/DllGetterRing.hpp"

using SensorBytes=std::array<unsigned char,52>;
using DoubleBytes=std::array<unsigned char,104>;
struct Wire {SensorBytes sensor{};std::uint64_t time{},qpc{};unsigned sequence{};};
struct Getter {SensorBytes sensor{};DoubleBytes original_sensor{};unsigned record{};};
struct Distribution {std::size_t zero{},one{},many{};void add(std::size_t n){if(!n)++zero;else if(n==1)++one;else ++many;}};
static bool read_exact(FILE*f,void*p,std::size_t n){return std::fread(p,1,n,f)==n;}
template<class Key>static void population(FILE*f,const std::map<Key,std::size_t>&m,std::size_t rows){
    std::size_t duplicate_groups=0,participating_rows=0,largest=0;
    for(const auto&x:m){if(x.second>1){++duplicate_groups;participating_rows+=x.second;}if(x.second>largest)largest=x.second;}
    std::fprintf(f,"{\"rows\":%zu,\"distinct_values\":%zu,\"duplicate_rows_beyond_first\":%zu,\"duplicate_groups\":%zu,\"rows_in_duplicate_groups\":%zu,\"largest_group\":%zu}",
        rows,m.size(),rows-m.size(),duplicate_groups,participating_rows,largest);
}
static void distribution(FILE*f,const Distribution&d){std::fprintf(f,"{\"zero\":%zu,\"unique\":%zu,\"multiple\":%zu}",d.zero,d.one,d.many);}
int wmain(int argc,wchar_t**argv){
    if((argc!=3&&argc!=4)||std::fesetround(FE_TONEAREST)!=0)return 2;
    const bool ring_input=argc==4;
    const bool matlab_reader=ring_input&&std::wstring(argv[3])==L"RDR1_MATLAB";
    if(ring_input&&!matlab_reader&&std::wstring(argv[3])!=L"RDR1")return 2;
    static_assert(sizeof(float)==4&&sizeof(double)==8,"Observed Windows ABI");
    static_assert(offsetof(mavlink_hil_sensor_t,xacc)==8&&offsetof(mavlink_hil_sensor_t,temperature)==56,"Actual MAVLink field mapping");
    const std::wstring input=argv[1];
    gpenmpc_clock_tap::SharedSection section{};
    std::vector<gpenmpc_clock_tap::DllGetterSample> records;
    LONG64 recorded_events=0,recorded_drops=0;LONG diagnostic_errors=0;bool retained_complete=false;
    FILE*f=nullptr;
    if(ring_input){
        gpenmpc_clock_ring::Section final{};
        f=_wfopen((input+L"\\GETTER_RING_SECTION_FINAL.bin").c_str(),L"rb");if(!f)return 3;
        const bool shape=read_exact(f,&final,sizeof final)&&std::fgetc(f)==EOF;std::fclose(f);
        const auto&h=final.header;
        if(!shape||!gpenmpc_clock_ring::shape(h)||h.consumed_read<0||h.consumed_read>8192)return 4;
        records.resize(static_cast<std::size_t>(h.consumed_read));
        f=_wfopen((input+(matlab_reader?L"\\MATLAB_GETTER_RECORDS.bin":L"\\GETTER_RING_RECORDS.bin")).c_str(),L"rb");if(!f)return 3;
        const bool exact=read_exact(f,records.data(),records.size()*sizeof(records[0]))&&std::fgetc(f)==EOF;std::fclose(f);
        if(!exact)return 4;
        recorded_events=h.events;recorded_drops=h.dropped;diagnostic_errors=h.sticky_errors;
        // Expect one explicit retire after capture and producer exit.
        // The external consumer closes separately and records its own receipt.
        const bool retirement=matlab_reader?(h.consumer_pid!=h.producer_pid&&h.consumer_pid>0&&
            h.sticky_errors==0&&h.consumer_retire_calls==0):
            (h.sticky_errors==gpenmpc_clock_ring::ConsumerGone&&h.consumer_retire_calls==1);
        retained_complete=h.events==h.published_write&&h.published_write==h.consumed_read&&h.dropped==0&&
            h.writer_busy==0&&retirement;
    }else{
        f=_wfopen((input+L"\\DLL_SHARED_SECTION.bin").c_str(),L"rb");if(!f)return 3;
        const bool section_ok=read_exact(f,&section,sizeof section)&&std::fgetc(f)==EOF;std::fclose(f);
        const auto&h=section.header;
        if(!section_ok||h.magic!=gpenmpc_clock_tap::section_magic||h.abi_version!=1||h.section_bytes!=sizeof section||
           h.record_bytes!=sizeof(gpenmpc_clock_tap::DllGetterSample)||h.capacity!=32||h.published_count<0||h.published_count>32)return 4;
        records.assign(section.records,section.records+h.published_count);
        recorded_events=h.events;recorded_drops=h.dropped;diagnostic_errors=h.sticky_errors;
        retained_complete=h.events==h.published_count&&h.dropped==0&&h.sticky_errors==0;
    }
    std::vector<Getter> getters;
    std::map<SensorBytes,std::size_t> converted_groups,wire_groups;
    std::map<DoubleBytes,std::size_t> original_groups;
    std::map<std::vector<unsigned char>,std::size_t> full_copied_groups;
    unsigned invalid_length=0,nonfinite=0,accepted_valid=0;
    double minimum_getter_time=INFINITY,maximum_getter_time=-INFINITY;
    for(std::size_t i=0;i<records.size();++i){const auto&r=records[i];
        accepted_valid+=r.observation_valid&&!r.model_failed&&!r.model_runtime_error;
        if(r.copied_length<14||r.copied_length>30){++invalid_length;continue;}
        if(r.hil_output30[0]<minimum_getter_time)minimum_getter_time=r.hil_output30[0];
        if(r.hil_output30[0]>maximum_getter_time)maximum_getter_time=r.hil_output30[0];
        const auto*raw=reinterpret_cast<const unsigned char*>(r.hil_output30);
        ++full_copied_groups[std::vector<unsigned char>(raw,raw+sizeof(double)*static_cast<std::size_t>(r.copied_length))];
        Getter g{};g.record=static_cast<unsigned>(i);
        std::memcpy(g.original_sensor.data(),&r.hil_output30[1],sizeof g.original_sensor);
        // NoUI 94B81EFB...: 0x14001c30f..c3cd converts exactly double[1:13]
        // into contiguous wire float[13] at payload offsets 8..59. It skips [0].
        // fields_updated is constructed separately; no accepted-step ID is added.
        bool finite=true;
        for(unsigned k=0;k<13;++k){const double d=r.hil_output30[k+1];
            volatile float converted=static_cast<float>(d);const float value=converted;
            if(!std::isfinite(d)||!std::isfinite(value))finite=false;
            std::memcpy(g.sensor.data()+4*k,&value,4);
        }
        if(!finite){++nonfinite;continue;}
        ++original_groups[g.original_sensor];++converted_groups[g.sensor];getters.push_back(g);
    }
    f=_wfopen((input+L"\\RAW_STREAMS.bin").c_str(),L"rb");if(!f)return 5;
    std::vector<Wire>wires;std::size_t raw_records=0,raw_bytes=0,bad_crc=0,bad_signature=0,frames=0;
    mavlink_message_t rx{},parsed{};mavlink_status_t parser{},reported{};
    for(;;){std::uint64_t qpc{};std::uint32_t channel{},length{};
        const auto got=std::fread(&qpc,1,8,f);if(got==0&&std::feof(f))break;
        if(got!=8||!read_exact(f,&channel,4)||!read_exact(f,&length,4)||channel>4||length>8U*1024U*1024U){std::fclose(f);return 6;}
        std::vector<unsigned char>bytes(length);if(!read_exact(f,bytes.data(),length)){std::fclose(f);return 6;}
        ++raw_records;raw_bytes+=length;if(raw_bytes>8U*1024U*1024U){std::fclose(f);return 6;}
        if(channel!=0)continue; // CopterSim-to-peer TCP stream.
        for(const auto byte:bytes){const auto status=mavlink_frame_char_buffer(&rx,&parser,byte,&parsed,&reported);
            bad_crc+=status==MAVLINK_FRAMING_BAD_CRC;bad_signature+=status==MAVLINK_FRAMING_BAD_SIGNATURE;
            if(status!=MAVLINK_FRAMING_OK)continue;++frames;
            if(parsed.msgid!=MAVLINK_MSG_ID_HIL_SENSOR)continue;
            mavlink_hil_sensor_t hil{};mavlink_msg_hil_sensor_decode(&parsed,&hil);
            Wire w{};w.qpc=qpc;w.time=hil.time_usec;w.sequence=parsed.seq;
            std::memcpy(w.sensor.data(),&hil.xacc,52);++wire_groups[w.sensor];wires.push_back(w);
        }
    }std::fclose(f);
    Distribution getter_matches,wire_matches;std::size_t total_pairs=0,mutually_unique=0;
    std::vector<std::size_t>getter_counts(getters.size()),wire_counts(wires.size());
    for(std::size_t g=0;g<getters.size();++g)for(std::size_t w=0;w<wires.size();++w)
        if(getters[g].sensor==wires[w].sensor){++getter_counts[g];++wire_counts[w];++total_pairs;}
    for(const auto n:getter_counts)getter_matches.add(n);
    for(const auto n:wire_counts)wire_matches.add(n);
    for(std::size_t g=0;g<getters.size();++g)if(getter_counts[g]==1)
        for(std::size_t w=0;w<wires.size();++w)if(getters[g].sensor==wires[w].sensor&&wire_counts[w]==1)++mutually_unique;
    // Byte-comparison controls, independent of the observed population.
    SensorBytes control{};float one=1.0f;std::memcpy(control.data(),&one,4);
    SensorBytes same=control,changed=control;changed[0]^=1;
    const bool byte_controls=(control==same)&&(control!=changed);
    f=_wfopen(argv[2],L"wb");if(!f)return 7;
    std::fprintf(f,"{\n\"scope\":\"OFFLINE_GETTER_FLOAT_CONTENT_COMPARISON\",\n");
    std::fprintf(f,"\"inputs\":[\"%s\",\"RAW_STREAMS.bin (original channel 0 TCP bytes; QPC retained, not used for matching)\"],\n",matlab_reader?"MATLAB_GETTER_RECORDS.bin plus GETTER_RING_SECTION_FINAL.bin (RDR1, before MATLAB close)":ring_input?"GETTER_RING_RECORDS.bin plus GETTER_RING_SECTION_FINAL.bin (RDR1)":"DLL_SHARED_SECTION.bin (original RDT1 10816-byte retained memory)");
    std::fprintf(f,"\"mapping\":\"NoUI SHA256 94B81EFB44058176DD5353669D9C28FC5331CC8411AB9EA3F2D27C1E8C343241; VA 0x14001c30f..0x14001c3cd: binary64 getter[1..13] to binary32 wire bytes[8..59], round-to-nearest; excluded time/fields_updated/id/sequence\",\n");
    std::fprintf(f,"\"getter_records\":%zu,\"eligible_getters\":%zu,\"invalid_length\":%u,\"nonfinite_conversion\":%u,\"accepted_valid_getters\":%u,\"events\":%lld,\"dropped\":%lld,\"sticky_errors\":%ld,\"complete_retained_getter_denominator\":%s,\n",
        records.size(),getters.size(),invalid_length,nonfinite,accepted_valid,recorded_events,recorded_drops,diagnostic_errors,retained_complete?"true":"false");
    std::fprintf(f,"\"getter_original_time_range_us\":[%.17g,%.17g],\"tcp_hil_sensor_frames\":%zu,\"all_tcp_valid_frames\":%zu,\"bad_crc\":%zu,\"bad_signature\":%zu,\"raw_records\":%zu,\"raw_payload_bytes\":%zu,\n",
        minimum_getter_time,maximum_getter_time,wires.size(),frames,bad_crc,bad_signature,raw_records,raw_bytes);
    std::fprintf(f,"\"getter_full_copied_double_population_including_time\":");population(f,full_copied_groups,getters.size());
    std::fprintf(f,",\n\"getter_original_13_double_sensor_population_excluding_time\":");population(f,original_groups,getters.size());
    std::fprintf(f,",\n\"getter_converted_13_float_sensor_population\":");population(f,converted_groups,getters.size());
    std::fprintf(f,",\n\"wire_13_float_sensor_population\":");population(f,wire_groups,wires.size());
    std::fprintf(f,",\n\"getters_classified_by_matching_wire_count\":");distribution(f,getter_matches);
    std::fprintf(f,",\n\"wires_classified_by_matching_getter_count\":");distribution(f,wire_matches);
    std::fprintf(f,",\n\"total_content_pairs\":%zu,\"mutually_unique_content_pairs\":%zu,\"byte_comparison_controls_pass\":%s,\n\"getter_rows\":[",total_pairs,mutually_unique,byte_controls?"true":"false");
    for(std::size_t g=0;g<getters.size();++g){const auto&r=records[getters[g].record];
        std::fprintf(f,"%s{\"retained_index\":%u,\"original_event_ordinal\":%llu,\"original_generation\":%llu,\"original_getter_time_us\":%.17g,\"matching_wire_count\":%zu}",
            g?",":"",getters[g].record,static_cast<unsigned long long>(r.event_ordinal),static_cast<unsigned long long>(r.accepted_generation),r.hil_output30[0],getter_counts[g]);
    }
    std::fprintf(f,"],\n\"mutually_unique_retained_pairs\":[");
    bool first_pair=true;
    for(std::size_t g=0;g<getters.size();++g)if(getter_counts[g]==1)
        for(std::size_t w=0;w<wires.size();++w)if(wire_counts[w]==1&&getters[g].sensor==wires[w].sensor){
            const auto&r=records[getters[g].record];const auto&wire=wires[w];
            std::fprintf(f,"%s{\"retained_getter_index\":%u,\"original_getter_event_ordinal\":%llu,\"original_getter_time_us\":%.17g,\"original_accepted_generation\":%llu,\"original_accepted_time_s\":%.17g,\"getter_observation_valid\":%u,\"original_getter_thread_id\":%u,\"wire_index\":%zu,\"original_wire_time_us\":%llu,\"original_wire_sequence\":%u,\"original_receive_record_qpc\":%llu,\"equal_sensor52_bytes_hex\":\"",
                first_pair?"":",",getters[g].record,static_cast<unsigned long long>(r.event_ordinal),r.hil_output30[0],
                static_cast<unsigned long long>(r.accepted_generation),r.accepted_time_s,r.observation_valid,r.getter_thread_id,
                w,static_cast<unsigned long long>(wire.time),wire.sequence,static_cast<unsigned long long>(wire.qpc));
            for(const auto b:wire.sensor)std::fprintf(f,"%02X",static_cast<unsigned>(b));
            std::fprintf(f,"\"}");first_pair=false;
        }
    std::fprintf(f,"],\n\"unique_source_association_proven\":false,\"model_getter_atomicity_proven\":false,\"receiver_hrt_association_proven\":false,\"new_simulator_runs\":0,\"COM\":0,\n\"limits\":\"Retained records are paired by exact content equality, with completeness reported separately. Multiple NoUI callsites share the getter; serialization and receiver-HRT association require independent observation.\"\n}\n");
    std::fclose(f);
    std::printf("getters=%zu wires=%zu pairs=%zu mutually_unique=%zu; getter 0/1/many=%zu/%zu/%zu; wire 0/1/many=%zu/%zu/%zu\n",getters.size(),wires.size(),total_pairs,mutually_unique,getter_matches.zero,getter_matches.one,getter_matches.many,wire_matches.zero,wire_matches.one,wire_matches.many);
    return byte_controls?0:8;
}
