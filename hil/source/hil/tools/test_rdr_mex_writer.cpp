// Test the MATLAB consumer with a mock Writer producer.
// Require unique test mapping names.
#include "../rfly_vendor_integration/clock_tap_overlay/DllStepSnapshotSharedStatus.hpp"
#include <string>
#include <cwchar>
int wmain(int argc,wchar_t**argv){
    if(argc!=4)return 2;const std::wstring mode=argv[3];
    if(mode!=L"normal"&&mode!=L"overflow"&&mode!=L"paced")return 2;
    if(!SetEnvironmentVariableW(gpenmpc_clock_ring::environment,argv[1])||
       !SetEnvironmentVariableW(gpenmpc_snapshot_status::environment,argv[2]))return 3;
    gpenmpc_clock_ring::Writer writer;writer.attach_once_at_initialize();
    gpenmpc_snapshot_status::Writer status;if(!writer.enabled()||!status.attach_once_after_ring_initialize())return 4;
    const unsigned count=mode==L"normal"?12:(mode==L"paced"?600:257);
    for(unsigned i=0;i<count;++i){gpenmpc_clock_ring::Sample s{};s.accepted_generation=i+1;s.plant_session=42;
        s.accepted_time_s=double(i)*.01;s.copied_length=30;s.observation_valid=1;
        for(unsigned k=0;k<30;++k)s.hil_output30[k]=double(i*100+k)*.125;
        for(unsigned k=0;k<6;++k)s.rotor_lag6_n[k]=double(i*10+k)*.25;
        writer.observe(s);std::uint64_t counters[10]={1,0,1,i+1,i+1,i+1,i+1,0,0,0};status.publish(counters);
        if(mode==L"paced")Sleep(1);
    }
    writer.destroy_requested();status.destroy_requested();return 0;
}
