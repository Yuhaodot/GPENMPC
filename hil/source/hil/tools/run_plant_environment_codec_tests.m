function r=run_plant_environment_codec_tests(outputJson)
% Test environment byte encoding and invalid inputs.
assert(~isfile(outputJson));root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'matlab_validation'));
v=struct('schema','M600_PLANT_ENVIRONMENT_V1','generation',uint32(2), ...
    'source_io_time_s',12.25,'task_reference_time_s',8.125,'payload_kg',2.21, ...
    'wind_ned_xy_mps',[-3;4],'mission_phase',1,'reference_jet_ned',(1:12)', ...
    'payload_generation',uint32(0),'task_clock_paused',false);
[b,f]=gpenmpcTaskIo.encodePlantEnvironment(v,1,2.21);checks=false(1,14);
checks(1)=numel(b)==232&&isa(b,'uint8');
checks(2)=isequal(b(1:4),uint8([217,2,150,73])); % independent checksum hex499602D9
checks(3)=isequal(b(5:8),uint8([1,0,0,0]));
checks(4)=isequal(b(9:16),uint8([0,0,0,0,0,0,240,63])); % version1.0
decoded=typecast(b(9:end),'double');[~,~,e]=computer;if e=='B',decoded=swapbytes(decoded);end
checks(5)=isequal(decoded,f)&&isequal(f(9:20),1:12);
checks(6)=all(f(23:28)==0)&&f(5)==2.21&&isequal(f(6:7),[-3,4]);
for k=1:8
    q=v;
    switch k
        case 1,q.generation=0;
        case 2,q.generation=1.5;
        case 3,q.payload_kg=2.22;
        case 4,q.payload_kg=-.01;
        case 5,q.reference_jet_ned(3)=NaN;
        case 6,q.wind_ned_xy_mps(1)=Inf;
        case 7,q.task_reference_time_s=-1;
        case 8,q.task_clock_paused=1; % must be explicit logical
    end
    try,gpenmpcTaskIo.encodePlantEnvironment(q,1,2.21);catch,checks(k+6)=true;end
end
r=struct('passed',all(checks),'checks_total',numel(checks),'checks_passed',sum(checks), ...
    'wire_bytes',numel(b),'fields',f,'hardware_actions',0,'socket_open',0, ...
    'runtime_environment_or_payload_change',false, ...
    'limitation','CODEC_ONLY__NO_DLL_INPUT_FRESHNESS_OR_PHYSICAL_UNLOAD_PROOF');
fid=fopen(outputJson,'w','n','UTF-8');assert(fid>=0);c=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(r,PrettyPrint=true));clear c;disp(jsonencode(r));
assert(r.passed);
end
