function report=run_terrain_diagnostic_dll_probe(newDllDir,oldDllDir,newOutputDir)
% Build the ABI probe with LLVM-MinGW and test the existing DLLs.
arguments
    newDllDir (1,1) string
    oldDllDir (1,1) string
    newOutputDir (1,1) string
end
assert(~isfolder(newOutputDir)&&~isfile(newOutputDir),'m600check:TerrainDllProbeExists');
b=string(fileparts(fileparts(mfilename('fullpath'))));oldPath=path;
pathGuard=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(b,'matlab_validation'),fullfile(b,'m600_coptersim','matlab_validation'),'-begin');
newDll=fullfile(newDllDir,'GPENMPC_M600_Canonical.dll');oldDll=fullfile(oldDllDir,'GPENMPC_M600_Canonical.dll');
newSha='458D2CCBFB50EC47E23E0F76AA9CDC851D14622A78B88205710060EC2BA3657D';
oldSha='80C95FA673EC462EB3108016A0F53FE4A7A50A7FD9F016DFB57EE9345A09EAE4';
assert(strcmpi(sha(newDll),newSha)&&strcmpi(sha(oldDll),oldSha),'m600check:TerrainDllIdentity');
compiler=gpenmpc_install_path('llvm','bin\clang++.exe');
source=fullfile(b,'tools','probe_terrain_diagnostic_dll.cpp');assert(isfile(compiler)&&isfile(source));
mkdir(newOutputDir);exe=fullfile(newOutputDir,'probe_terrain_diagnostic_dll.exe');
csv=fullfile(newOutputDir,'TERRAIN_DLL_TRACE.csv');raw=fullfile(newOutputDir,'PAIRED_RAW_OUTPUTS.bin');
report=struct('schema','HOST_TERRAIN_EXTENSION_GENERATED_DLL_PROBE_V1','passed',false, ...
    'status','PROBE_NOT_COMPLETED','failure','','new_dll',newDll,'new_dll_sha256',newSha, ...
    'old_dll',oldDll,'old_dll_sha256',oldSha,'probe_source',source,'probe_source_sha256',sha(source), ...
    'wrapper_source',string(mfilename('fullpath'))+".m",'compiler',compiler, ...
    'compile_returncode',NaN,'probe_returncode',NaN,'rows',0,'cases',0, ...
    'checks',struct('name',{},'passed',{}),'first_faults',struct([]), ...
    'arithmetic_absolute_tolerance',1e-9, ...
    'tolerance_provenance','EXISTING_SIMULINK_SOURCE_REPLAY_ARITHMETIC__NOT_FLIGHT_SCREEN', ...
    'diagnostic_prefix_requirement','FIRST_SEVEN_DOUBLES_BYTE_EXACT', ...
    'raw_format','LE_HEADER_4UINT32_THEN_RECORD_2UINT32_AND_319DOUBLE__2560_BYTES_PER_RECORD', ...
    'generated_model_rebuilt',false,'model_loaded',false, ...
    'COM_open',0,'UDP_open',0,'board_actions',0,'CopterSim_started',false, ...
    'controller_closed_loop',false,'HIL_performance_claim',false,'terrain_source_cause_identified',false);
try
    compile=q(compiler)+" -std=c++17 -O2 -static -municode "+q(source)+" -o "+q(exe);
    report.compile_command=compile;
    [rc,log]=system(compile);report.compile_returncode=rc;writeText(fullfile(newOutputDir,'COMPILE.log'),log);
    assert(rc==0,'m600check:TerrainProbeCompile','%s',log);
    command=q(exe)+" "+q(oldDll)+" "+q(newDll)+" "+q(csv)+" "+q(raw);report.probe_command=command;
    [rc,log]=system(command);report.probe_returncode=rc;writeText(fullfile(newOutputDir,'PROBE.log'),log);
    t=readtable(csv,'VariableNamingRule','preserve','TextType','string');
    report.rows=height(t);report.cases=numel(unique(t.case_index));
    check('complete_50_cases_5940_rows',height(t)==5940&&report.cases==50);
    check('CXX_all_row_invariants_and_exit',rc==0&&all(t.issue_mask==0));
    f=fopen(raw,'rb');assert(f>=0);fileGuard=onCleanup(@()fclose(f)); %#ok<NASGU>
    head=fread(f,4,'*uint32');check('raw_header_exact',isequal(head,uint32([hex2dec('4d365450');1;2560;319])));
    firstRows=cell(0,1);previousCase=0;previousTime=NaN;firstCapture=[];firstInput=[];
    prefixOkay=true;rawCaptureOkay=true;decoderOkay=true;allVectorsFinite=true;pairMax=zeros(1,3);
    recordOkay=true;resetOkay=true;trailingOkay=true;bitwiseRows=zeros(1,3);
    for row=1:height(t)
        meta=fread(f,2,'*uint32');values=fread(f,319,'*double');
        assert(numel(meta)==2&&numel(values)==319,'m600check:TerrainRawTruncated','row=%d',row);
        recordOkay=recordOkay&&meta(1)==t.case_index(row)&&meta(2)==t.step(row);
        input=values(1:15);a=values(16:167);d=values(168:319);pa=a(121:152);pd=d(121:152);
        if t.case_index(row)~=previousCase
            previousCase=t.case_index(row);previousTime=NaN;firstCapture=[];firstInput=[];
            resetOkay=resetOkay&&t.step(row)==0&&pd(3)==0&&abs(d(3)-.001)<1e-12;
            if t.expected_first_fault(row)~=0,resetOkay=resetOkay&&pd(1)==0&&all(pd(8:25)==0);end
        end
        prefixOkay=prefixOkay&&sameBits(pa(1:7),pd(1:7));
        decoded=m600check.decodeCopterSimTerrainDiagnostics(packet(pd),1,previousTime);
        decoderOkay=decoderOkay&&decoded.packet_valid&&decoded.terrain_extension.valid&& ...
            decoded.must_stop==(pd(1)~=0);previousTime=pd(3);
        if pd(1)==1
            if isempty(firstCapture)
                firstCapture=pd(8:25);firstInput=input;
                firstRows{end+1,1}=struct('case_name',t.case_name(row),'first_step',t.step(row), ...
                    'first_reason',pd(9),'locked_height',pd(10), ...
                    'input_float64_hex',{hexVector(input)},'captured_float64_hex',{hexVector(pd(11:25))}); %#ok<AGROW>
            end
            rawCaptureOkay=rawCaptureOkay&&sameBits(pd(8:25),firstCapture)&&sameBits(pd(11:25),firstInput)&& ...
                sameBits(decoded.terrain_extension.first_terrain15,firstInput)&&decoded.must_stop;
        else
            rawCaptureOkay=rawCaptureOkay&&isempty(firstCapture)&&all(pd(8:25)==0);
        end
        groups={1:60,61:90,91:120};
        for j=1:3
            ii=groups{j};finite=all(isfinite(a(ii)))&&all(isfinite(d(ii)));allVectorsFinite=allVectorsFinite&&finite;
            if finite,pairMax(j)=max(pairMax(j),max(abs(a(ii)-d(ii))));else,pairMax(j)=Inf;end
            bitwiseRows(j)=bitwiseRows(j)+double(sameBits(a(ii),d(ii)));
        end
    end
    trailingOkay=isempty(fread(f,1,'*uint8'));clear fileGuard
    check('all_raw_record_ids_and_no_trailing_bytes',recordOkay&&trailingOkay);
    check('independent_raw_legacy_prefix7_bit_exact',prefixOkay);
    check('independent_raw_capture_all15_first_fault_no_wash',rawCaptureOkay);
    check('independent_raw_decoder_all_packets_fail_closed',decoderOkay);
    check('each_explicit_reset_and_new_instance_starts_zero',resetOkay);
    check('all_paired_original_plant_sensor_GPS_finite_and_1e_minus9',allVectorsFinite&&all(pairMax<=1e-9));
    check('45_channel_faults_plus_height_and_cold_fault',numel(firstRows)==47);
    check('reloaded_instance_observed_only_in_last_case', ...
        all(t.instance_generation(t.case_index<50)==1)&&all(t.instance_generation(t.case_index==50)==2));
    report.first_faults=vertcat(firstRows{:});report.paired_max_abs_difference_vehicle_sensor_GPS=pairMax;
    report.bitwise_equal_rows_vehicle_sensor_GPS=bitwiseRows;
    check('both_DLL_bytes_unchanged',strcmpi(sha(newDll),newSha)&&strcmpi(sha(oldDll),oldSha));
    report.passed=true;report.status='PASS_GENERATED_DLL_TERRAIN_EXTENSION_AND_PARENT_OUTPUT_EQUIVALENCE__HOST_ONLY';
catch exception
    report.failure=getReport(exception,'extended','hyperlinks','off');
    report.status='HOST_DLL_TERRAIN_PROBE_FAILED__NOT_HARDWARE_RESULT';
end
report.checks_total=numel(report.checks);report.checks_passed=sum([report.checks.passed]);
report.wrapper_sha256=sha(report.wrapper_source);report.evidence=struct('path',{},'bytes',{},'sha256',{});
for file=[csv,raw,exe,fullfile(newOutputDir,'COMPILE.log'),fullfile(newOutputDir,'PROBE.log')]
    if isfile(file),info=dir(file);report.evidence(end+1)=struct('path',file,'bytes',info.bytes,'sha256',sha(file));end %#ok<AGROW>
end
writeText(fullfile(newOutputDir,'RESULT.json'),jsonencode(report,PrettyPrint=true));
disp(struct('passed',report.passed,'rows',report.rows,'cases',report.cases,'COM_open',0));
assert(report.passed,'m600check:TerrainDllProbeFailed','%s',report.failure);
    function check(name,passed)
        passed=isscalar(passed)&&logical(passed);report.checks(end+1)=struct('name',name,'passed',passed);
        assert(passed,'m600check:TerrainDllProbeCheck','Failed: %s',name);
    end
end
function yes=sameBits(a,b),yes=isequal(size(a),size(b))&&isequal(typecast(a(:),'uint64'),typecast(b(:),'uint64'));end
function h=hexVector(a),h=cellstr(upper(dec2hex(typecast(a(:),'uint64'),16))).';end
function bytes=packet(payload)
head=int32([1234567890;1]);values=payload(:);[~,~,endian]=computer;
if endian=='B',head=swapbytes(head);values=swapbytes(values);end
bytes=[typecast(head,'uint8');typecast(values,'uint8')];bytes=bytes(:);
end
function h=sha(file),h=m600check.fileSha256(file);end
function s=q(x),assert(~contains(x,'"'));s='"'+string(x)+'"';end
function writeText(file,text)
f=fopen(file,'w','n','UTF-8');assert(f>=0);guard=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',text);
end
