function report=run_m600_terrain_diagnostic_extension_tests(outputDir)
% Test terrain diagnostic codecs and state handling.
arguments,outputDir (1,1) string = "",end
if strlength(outputDir)>0
    assert(~isfolder(outputDir)&&~isfile(outputDir),'m600check:TestOutputExists','Preserve earlier evidence.');
end
build=string(fileparts(fileparts(mfilename('fullpath'))));oldPath=path;
guard=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(build,'matlab_validation'),'-begin');
addpath(fullfile(build,'m600_coptersim','matlab_validation'),'-end');
checks=struct('name',{},'passed',{});examples=struct('name',{},'packet_hex',{},'decoded',{});
healthy=[0;0;1.25;0;1;0;1];failed=[1;4;1.25;2;0;1;1];terrain=zeros(15,1);
legacy=zeros(32,1);legacy(1:7)=healthy;legacyBytes=packet(legacy);
old=m600check.decodeCopterSimDiagnostics(legacyBytes,1,1);
decoded=m600check.decodeCopterSimTerrainDiagnostics(legacyBytes,1,1,false);
fields=fieldnames(old);same=true;
for k=1:numel(fields),same=same&&isequaln(old.(fields{k}),decoded.(fields{k}));end
check('explicit_legacy_mode_preserves_all_old_decoder_fields_and_bytes',same&&isequal(decoded.raw_datagram,legacyBytes));
check('new_model_requires_extension_not_legacy_disguised',stops(m600check.decodeCopterSimTerrainDiagnostics(legacyBytes,1,1)));

state=m600check.initialCopterSimTerrainDiagnosticState();
[p,state]=m600check.encodeCopterSimTerrainDiagnostics(healthy,terrain,0,0,false,true,state);
check('prefix7_bit_exact_extension_is_18_data_plus_tag', ...
    sameBits(p(1:7),healthy)&&all(p(8:25)==0)&&p(26)==1&&all(p(27:32)==0));
decoded=decode(p);check('healthy_extension_preserves_original_health_semantics', ...
    decoded.packet_valid&&decoded.terrain_extension.valid&&~decoded.must_stop&&decoded.can_use_as_healthy_observation);
originalState=state;
changed=terrain;changed(1)=1+eps(1);
[p,state]=m600check.encodeCopterSimTerrainDiagnostics(failed,changed,3,1,true,false,state);
decoded=decode(p);
check('one_ulp_finite_height_change_is_preserved_and_always_stops', ...
    decoded.packet_valid&&decoded.must_stop&&~decoded.can_use_as_healthy_observation&& ...
    decoded.terrain_extension.first_reason==3&&decoded.terrain_extension.first_locked_height==1&& ...
    sameBits(decoded.terrain_extension.first_terrain15,changed));
saveExample('FIRST_FINITE_HEIGHT_CHANGE',p,decoded);
first=state;later=terrain;later(15)=Inf;
[latched,state]=m600check.encodeCopterSimTerrainDiagnostics(failed,later,2,17,true,false,state);
check('subsequent_fault_cannot_overwrite_first_input_reason_height', ...
    isequaln(state,first)&&sameBits(latched(8:32),p(8:32)));
[latched,state]=m600check.encodeCopterSimTerrainDiagnostics(failed,later,2,17,true,true,state);
check('invalid_reset_while_terrain_fault_latched_cannot_clear_first_capture', ...
    isequaln(state,first)&&sameBits(latched(8:32),p(8:32)));
[latched,state]=m600check.encodeCopterSimTerrainDiagnostics(failed,terrain,3,0,true,false,state);
decoded=decode(latched);
check('later_good_raw_without_accepted_reset_keeps_first_fault',isequaln(state,first)&&decoded.must_stop);
[p,state]=m600check.encodeCopterSimTerrainDiagnostics(healthy,terrain,0,0,false,true,state);
decoded=decode(p);
check('accepted_reset_clears_only_observer_latch',isequaln(state,originalState)&&~decoded.must_stop);

nanBits=bitor(bitshift(uint64(hex2dec('7FF80000')),32),uint64(1));
negativeNanBits=bitor(bitshift(uint64(hex2dec('FFF80000')),32),uint64(123));
injected={typecast(nanBits,'double'),Inf,-Inf,typecast(negativeNanBits,'double')};
labels={'NAN_PAYLOAD_1','PLUS_INF','MINUS_INF','NEGATIVE_NAN_PAYLOAD_123'};
for channel=1:15
    for kind=1:numel(injected)
        raw=zeros(15,1);raw(channel)=injected{kind};
        state=m600check.initialCopterSimTerrainDiagnosticState();
        [p,state]=m600check.encodeCopterSimTerrainDiagnostics(failed,raw,2,0,true,false,state); %#ok<ASGLU>
        decoded=decode(p);
        check(sprintf('channel_%02d_%s_roundtrip_exact_and_fail_closed',channel,labels{kind}), ...
            decoded.packet_valid&&decoded.terrain_extension.valid&&decoded.must_stop&& ...
            ~decoded.can_use_as_healthy_observation&&decoded.terrain_extension.first_reason==2&& ...
            sameBits(p(1:7),failed)&&sameBits(decoded.terrain_extension.first_terrain15,raw)&& ...
            isequal(decoded.terrain_extension.first_terrain_float64_hex,hexVector(raw)));
        if channel==15,saveExample(labels{kind},p,decoded);end
    end
end
state=m600check.initialCopterSimTerrainDiagnosticState();
[p,~]=m600check.encodeCopterSimTerrainDiagnostics(failed,terrain,1,0,true,false,state);
decoded=decode(p);
check('cold_no_reset_reason1_requires_finite_raw_and_stops',decoded.packet_valid&&decoded.must_stop);
base=p;
for kind={'tag_zero','tag_unknown','tag_nan','reserved_nonzero','reserved_nan', ...
        'capture_nonbinary','reason_zero','reason_fraction','reason_unknown','locked_nan', ...
        'nonfinite_reason_but_finite_raw','changed_reason_but_same_height', ...
        'reason1_with_nonfinite_raw','healthy_prefix_with_capture','no_capture_with_code4', ...
        'no_capture_with_nonzero_reason','no_capture_with_nonzero_locked','no_capture_with_nonzero_raw'}
    p=base;
    switch kind{1}
        case 'tag_zero',p(26)=0;
        case 'tag_unknown',p(26)=2;
        case 'tag_nan',p(26)=NaN;
        case 'reserved_nonzero',p(32)=1;
        case 'reserved_nan',p(32)=NaN;
        case 'capture_nonbinary',p(8)=.5;
        case 'reason_zero',p(9)=0;
        case 'reason_fraction',p(9)=1.5;
        case 'reason_unknown',p(9)=4;
        case 'locked_nan',p(10)=NaN;
        case 'nonfinite_reason_but_finite_raw',p(9)=2;
        case 'changed_reason_but_same_height',p(9)=3;
        case 'reason1_with_nonfinite_raw',p(11)=Inf;
        case 'healthy_prefix_with_capture',p(1:7)=healthy;
        case 'no_capture_with_code4',p(8:25)=0;
        case 'no_capture_with_nonzero_reason',p(1:7)=healthy;p(8)=0;
        case 'no_capture_with_nonzero_locked',p(1:7)=healthy;p(8:25)=0;p(10)=2;
        case 'no_capture_with_nonzero_raw',p(1:7)=healthy;p(8:25)=0;p(11)=2;
    end
    check(['invalid_' kind{1} '_fail_closed'],stops(decode(p)));
end
bytes=packet(base);
check('short_packet_fail_closed',stops(m600check.decodeCopterSimTerrainDiagnostics(bytes(1:263),1,1)));
check('long_packet_fail_closed',stops(m600check.decodeCopterSimTerrainDiagnostics([bytes;uint8(0)],1,1)));
check('matrix_byte_shape_fail_closed',stops(m600check.decodeCopterSimTerrainDiagnostics(reshape(bytes,8,33),1,1)));
check('wrong_byte_type_fail_closed',stops(m600check.decodeCopterSimTerrainDiagnostics(double(bytes),1,1)));
bad=bytes;bad(1)=bitxor(bad(1),uint8(1));
check('checksum_fail_closed',stops(m600check.decodeCopterSimTerrainDiagnostics(bad,1,1)));
check('copter_identity_fail_closed',stops(m600check.decodeCopterSimTerrainDiagnostics(bytes,2,1)));
p=zeros(32,1);p(1:7)=healthy;p(26)=1;
decoded=m600check.decodeCopterSimTerrainDiagnostics(packet(p),1,2);
check('source_time_reversal_never_gets_health_credit', ...
    decoded.must_stop&&decoded.packet_valid&&~decoded.can_use_as_healthy_observation);
for index=1:7
    bad=p;bad(index)=NaN;
    check(sprintf('nonfinite_prefix_field_%d_still_rejected',index),stops(decode(bad)));
end
bad=p;bad(7)=2;
check('prefix_version_not_silently_changed',stops(decode(bad)));
% Reject nonzero unknown reserved payloads in legacy-compatible mode.
bad=legacy;bad(31)=7;
check('unknown_legacy_reserved_payload_still_rejected', ...
    stops(m600check.decodeCopterSimTerrainDiagnostics(packet(bad),1,1,false)));
% Fixed-size encoder contract and raw signed-zero are separately checked.
state=m600check.initialCopterSimTerrainDiagnosticState();
check('encoder_prefix_shape_rejected',throws(@()m600check.encodeCopterSimTerrainDiagnostics( ...
    healthy.',terrain,0,0,false,true,state)));
check('encoder_terrain_shape_rejected',throws(@()m600check.encodeCopterSimTerrainDiagnostics( ...
    healthy,terrain.',0,0,false,true,state)));
check('encoder_wrong_numeric_class_rejected',throws(@()m600check.encodeCopterSimTerrainDiagnostics( ...
    single(healthy),terrain,0,0,false,true,state)));
check('encoder_complex_terrain_rejected',throws(@()m600check.encodeCopterSimTerrainDiagnostics( ...
    healthy,complex(terrain,ones(15,1)),0,0,false,true,state)));
badState=state;badState.first_terrain15=zeros(14,1);
check('encoder_state_shape_rejected',throws(@()m600check.encodeCopterSimTerrainDiagnostics( ...
    healthy,terrain,0,0,false,true,badState)));
raw=terrain;raw(1)=typecast(bitshift(uint64(1),63),'double');raw(15)=Inf;
[p,~]=m600check.encodeCopterSimTerrainDiagnostics(failed,raw,2,0,true,false,state);
decoded=decode(p);
check('signed_zero_raw_bits_not_normalized',decoded.packet_valid&&decoded.must_stop&& ...
    sameBits(decoded.terrain_extension.first_terrain15,raw));
nonTerrain=healthy;nonTerrain(1:2)=[1;2];
[p,~]=m600check.encodeCopterSimTerrainDiagnostics(nonTerrain,terrain,0,0,false,false,state);
decoded=decode(p);
check('nonterrain_model_fault_retains_prefix_fail_closed_without_fake_terrain_reason', ...
    decoded.packet_valid&&decoded.must_stop&&decoded.failure_code==2&& ...
    ~decoded.terrain_extension.capture_valid&&decoded.terrain_extension.first_reason==0);
report=struct('schema','HOST_TERRAIN_DIAGNOSTIC_EXTENSION_TEST_V1','passed',all([checks.passed]), ...
    'checks_total',numel(checks),'checks_passed',sum([checks.passed]),'checks',checks,'examples',examples, ...
    'COM_open',0,'UDP_open',0,'DLL_loaded',false,'model_started',false,'board_actions',0, ...
    'terrain_source_cause_identified',false);
sources={'m600check.initialCopterSimTerrainDiagnosticState','m600check.encodeCopterSimTerrainDiagnostics', ...
    'm600check.decodeCopterSimTerrainDiagnostics','m600check.decodeCopterSimDiagnostics',mfilename};
report.source_bindings=struct('path',{},'bytes',{},'sha256',{});
for k=1:numel(sources)
    source=which(sources{k});info=dir(source);assert(isscalar(info));
    report.source_bindings(end+1)=struct('path',source,'bytes',info.bytes, ...
        'sha256',m600check.fileSha256(source)); %#ok<AGROW>
end
if strlength(outputDir)>0
    mkdir(outputDir);save(fullfile(outputDir,'RAW_CODEC_TEST.mat'),'report','-v7');
    fid=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(fid>=0);
    fileGuard=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));clear fileGuard
end
disp(struct('passed',report.passed,'checks_total',report.checks_total,'checks_passed',report.checks_passed));

    function decoded=decode(payload)
        decoded=m600check.decodeCopterSimTerrainDiagnostics(packet(payload),1,1);
    end
    function check(name,okay)
        okay=isscalar(okay)&&logical(okay);
        checks(end+1)=struct('name',name,'passed',okay); %#ok<AGROW>
        assert(okay,'m600check:TerrainExtensionTest','Failed: %s',name);
    end
    function saveExample(name,payload,decoded)
        examples(end+1)=struct('name',name,'packet_hex',upper(reshape(dec2hex(packet(payload),2).',1,[])), ...
            'decoded',decoded); %#ok<AGROW>
    end
end
function bytes=packet(payload)
head=int32([1234567890;1]);values=payload(:);[~,~,endian]=computer;
if endian=='B',head=swapbytes(head);values=swapbytes(values);end
bytes=[typecast(head,'uint8');typecast(values,'uint8')];bytes=bytes(:);
end
function yes=sameBits(a,b),yes=isequal(typecast(double(a(:)),'uint64'),typecast(double(b(:)),'uint64'));end
function h=hexVector(a),h=cellstr(upper(dec2hex(typecast(double(a(:)),'uint64'),16))).';end
function yes=stops(r),yes=~r.packet_valid&&r.must_stop&&~r.can_use_as_healthy_observation;end
function yes=throws(f)
yes=false;try,f();catch,yes=true;end
end
