function report = runDiagnosticDecodeTests()
%RUNDIAGNOSTICDECODETESTS Pure packet fixtures; does not create a UDP socket.
rows=struct('name',{},'passed',{},'status',{});
payload=zeros(32,1);payload(3)=1.25;payload(4)=93.163175;payload(5)=1;payload(7)=1;
bytes=packet(payload,1,1234567890);
r=m600check.decodeCopterSimDiagnostics(bytes,1,1.24);
assert(r.packet_valid &&r.can_use_as_healthy_observation &&~r.must_stop);
assert(r.sim_time_s==1.25 &&r.contact_force_n==payload(4) &&r.ground_confirmed);
rows(end+1)=row('valid_official_v1_packet',r);
r=m600check.decodeCopterSimDiagnostics(bytes.',1,NaN);
assert(r.can_use_as_healthy_observation);
rows(end+1)=row('row_bytes_initial_run',r);
r=m600check.decodeCopterSimDiagnostics(bytes,1,1.25);
assert(r.can_use_as_healthy_observation &&r.time_monotonic);
rows(end+1)=row('duplicate_timestamp_not_reverse',r);
for length=[0,8,263,265]
    short=zeros(length,1,'uint8');
    r=m600check.decodeCopterSimDiagnostics(short,1,NaN);
    assert(~r.packet_valid &&r.must_stop);
    rows(end+1)=row(sprintf('reject_length_%d',length),r); %#ok<AGROW>
end
r=m600check.decodeCopterSimDiagnostics(double(bytes),1,NaN);
assert(~r.packet_valid &&r.must_stop);rows(end+1)=row('reject_nonuint8',r);
r=m600check.decodeCopterSimDiagnostics(packet(payload,2,1234567890),1,NaN);
assert(~r.packet_valid &&r.must_stop &&strcmp(r.status,'WRONG_COPTER_ID'));
rows(end+1)=row('reject_copter_identity',r);
r=m600check.decodeCopterSimDiagnostics(packet(payload,1,123456789),1,NaN);
assert(~r.packet_valid &&r.must_stop);rows(end+1)=row('reject_other_official_packet_checksum',r);
for index=[1,3,4,8]
    bad=payload;bad(index)=NaN;
    r=m600check.decodeCopterSimDiagnostics(packet(bad,1,1234567890),1,NaN);
    assert(~r.packet_valid &&r.must_stop);
    rows(end+1)=row(sprintf('reject_nan_field_%d',index),r); %#ok<AGROW>
end
bad=payload;bad(7)=2;
r=m600check.decodeCopterSimDiagnostics(packet(bad,1,1234567890),1,NaN);
assert(~r.packet_valid &&r.must_stop);rows(end+1)=row('reject_unknown_version',r);
bad=payload;bad(32)=1;
r=m600check.decodeCopterSimDiagnostics(packet(bad,1,1234567890),1,NaN);
assert(~r.packet_valid &&r.must_stop);rows(end+1)=row('reject_nonzero_reserved',r);
bad=payload;bad(1)=1;bad(2)=3;
r=m600check.decodeCopterSimDiagnostics(packet(bad,1,1234567890),1,1.25);
assert(r.packet_valid &&r.model_failed &&r.must_stop &&~r.can_use_as_healthy_observation);
rows(end+1)=row('model_failure_is_valid_packet_but_mandatory_abort',r);
r=m600check.decodeCopterSimDiagnostics(bytes,1,1.26);
assert(r.packet_valid &&~r.time_monotonic &&r.must_stop);
rows(end+1)=row('time_reverse_mandatory_abort',r);
bad=payload;bad(2)=3;
r=m600check.decodeCopterSimDiagnostics(packet(bad,1,1234567890),1,NaN);
assert(~r.packet_valid &&r.must_stop);rows(end+1)=row('reject_code_without_failure',r);
bad=payload;bad(1)=1;
r=m600check.decodeCopterSimDiagnostics(packet(bad,1,1234567890),1,NaN);
assert(~r.packet_valid &&r.must_stop);rows(end+1)=row('reject_failure_without_code',r);
bad=payload;bad(5)=.5;
r=m600check.decodeCopterSimDiagnostics(packet(bad,1,1234567890),1,NaN);
assert(~r.packet_valid &&r.must_stop);rows(end+1)=row('reject_nonbinary_ground_flag',r);
report=struct('status','PASS_OFFICIAL_OUTCOPTERDATA_DIAGNOSTIC_DECODER_FIXTURES', ...
    'test_count',numel(rows),'passed_count',nnz([rows.passed]),'tests',rows, ...
    'hardware_actions',0,'socket_open_count',0,'live_delivery_proven',false);
disp(jsonencode(report,PrettyPrint=true));
end

function bytes=packet(payload,id,checksum)
header=int32([checksum;id]);data=double(payload(:));
[~,~,endian]=computer;
if endian=='B'
    header=swapbytes(header);data=swapbytes(data);
end
a=typecast(header,'uint8');b=typecast(data,'uint8');
bytes=[a(:);b(:)];
end

function r=row(name,result)
r=struct('name',name,'passed',true,'status',result.status);
end
