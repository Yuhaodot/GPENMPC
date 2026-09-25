function report=test_rfly_original_getter_mex(outputRoot)
% Test Windows shared-memory consumption with a mock producer.
arguments,outputRoot (1,1) string,end
build=string(fileparts(fileparts(mfilename('fullpath'))));assert(~isfolder(outputRoot));mkdir(outputRoot);
oldPath=path;restore=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(build,'host_runtime'));diary(fullfile(outputRoot,'MATLAB_DIARY.txt'));
dg=onCleanup(@()diary('off')); %#ok<NASGU>
cc=gpenmpc_install_path('llvm','bin\clang.exe');
cxx=gpenmpc_install_path('llvm','bin\clang++.exe');
mr=matlabroot;tools=fullfile(build,'tools');log="";commands=strings(0,1);
version=fullfile(mr,'extern','version','c_mexapi_version.c');vo=fullfile(outputRoot,'version.o');
run(sprintf('"%s" -std=c11 -O2 -DMATLAB_MEX_FILE -DMATLAB_DEFAULT_RELEASE=R2018a -I"%s" -c "%s" -o "%s"',cc,fullfile(mr,'extern','include'),version,vo));
bin=fullfile(outputRoot,'rfly_original_getter_mex.mexw64');
run(sprintf('"%s" -std=c++14 -O2 -shared -static -Wall -Wextra -Werror -DMATLAB_MEX_FILE -DMATLAB_DEFAULT_RELEASE=R2018a -I"%s" "%s" "%s" "%s" "%s" "%s" -Wl,--no-undefined -o "%s"', ...
    cxx,fullfile(mr,'extern','include'),fullfile(tools,'rfly_original_getter_mex.cpp'),vo, ...
    fullfile(mr,'extern','lib','win64','microsoft','libmex.lib'),fullfile(mr,'extern','lib','win64','microsoft','libmx.lib'), ...
    fullfile(mr,'extern','lib','win64','mingw64','exportsmexfileversion.def'),bin));
writer=fullfile(outputRoot,'mock_writer.exe');
run(sprintf('"%s" -std=c++14 -O2 -static -municode -Wall -Wextra -Werror "%s" -o "%s"',cxx,fullfile(tools,'test_rdr_mex_writer.cpp'),writer));
addpath(outputRoot,'-begin');cleanup=onCleanup(@closeOwner); %#ok<NASGU>
checks=struct('name',{},'pass',{});
opened=rfly_original_getter_mex('open');
check('new_unique_sections_empty',~opened.failed&&isempty(opened.records)&&opened.total_read==0 ...
    &&numel(opened.ring_header)==160&&numel(opened.step_status)==144&&~opened.board_authority&&~opened.snapshot_coherent);
check('no_second_consumer',reject(@()rfly_original_getter_mex('open')));
before=gpenmpcNative.rflyOriginalHostMonotonicNs();
run(sprintf('"%s" "%s" "%s" normal',writer,opened.ring_name,opened.status_name));
first=rfly_original_getter_mex('drain');after=gpenmpcNative.rflyOriginalHostMonotonicNs();
check('real_ring_twelve_records',~first.failed&&isequal(size(first.records),[336,12])&&first.total_read==12);
check('qpc_same_original_stopwatch_domain',all(first.original_read_ns>=before&first.original_read_ns<=after));
for k=1:12
    b=first.records(:,k);event=typecast(b(1:8),'uint64');generation=typecast(b(9:16),'uint64');
    hil=typecast(b(25:264),'double');lag=typecast(b(273:320),'double');
    check(sprintf('unaltered_original_record_%02d',k),event==k&&generation==k ...
        &&isequal(hil,((k-1)*100+(0:29))'*.125)&&isequal(lag,((k-1)*10+(0:5))'*.25));
end
empty=rfly_original_getter_mex('drain');check('records_retired_not_replayed',isempty(empty.records)&&empty.total_read==12&&~empty.failed);
closed=rfly_original_getter_mex('close');check('closed_keeps_counts',closed.total_read==12&&~closed.failed);
check('closed_does_not_reopen',reject(@()rfly_original_getter_mex('drain')));
pacedOpened=rfly_original_getter_mex('open');check('explicit_new_session_unique',~strcmp(pacedOpened.ring_name,opened.ring_name)&&pacedOpened.peer_nonce~=opened.peer_nonce);
% MATLAB's main thread is deliberately blocked while the existing consumer
% continuously drains more than the producer's 256-record capacity.
run(sprintf('"%s" "%s" "%s" paced',writer,pacedOpened.ring_name,pacedOpened.status_name));
unblocked=gpenmpcNative.rflyOriginalHostMonotonicNs();paced=cell(3,1);
for k=1:3,paced{k}=rfly_original_getter_mex('drain');end
rows=cat(2,paced{1}.records,paced{2}.records,paced{3}.records);
times=cat(1,paced{1}.original_read_ns,paced{2}.original_read_ns,paced{3}.original_read_ns);
check('blocked_matlab_preserves_all_600_ordered_records',size(rows,2)==600&&all(cellfun(@(v)~v.failed,paced)) ...
    &&isequal(typecast(reshape(rows(1:8,:),[],1),'uint64'),uint64((1:600)')));
check('read_times_are_background_reads_not_later_matlab_drain',numel(times)==600&&all(times<=unblocked)&&all(diff(times)>0));
check('producer_ring_overflow_remains_zero',typecast(paced{3}.ring_header(97:104),'uint64')==0);
pacedClose=rfly_original_getter_mex('close');check('worker_close_preserves_count',pacedClose.total_read==600&&~pacedClose.failed);
save(fullfile(outputRoot,'RAW.mat'),'opened','first','empty','closed','pacedOpened','paced','pacedClose');
tracked=[string(mfilename('fullpath'))+'.m';fullfile(tools,'rfly_original_getter_mex.cpp');fullfile(tools,'test_rdr_mex_writer.cpp'); ...
    fullfile(build,'rfly_vendor_integration','clock_tap_overlay','DllGetterRing.hpp');fullfile(build,'rfly_vendor_integration','clock_tap_overlay','DllStepSnapshotSharedStatus.hpp');bin;writer];
report=struct('scope','MATLAB_REAL_WINDOWS_ORIGINAL_RDR_SSS_CONSUMER_WITH_EXPLICIT_MOCK_WRITER', ...
    'checks',checks,'total',numel(checks),'passed',sum([checks.pass]),'all_pass',all([checks.pass]), ...
    'commands',commands,'source_paths',tracked,'sha256',arrayfun(@sha,tracked), ...
    'hardware_actions',0,'COM',0,'outputs',0);
write(fullfile(outputRoot,'RESULT.json'),jsonencode(report,PrettyPrint=true));
fprintf('Original RDR MATLAB consumer %d/%d\n',report.passed,report.total);assert(report.all_pass);
    function run(cmd)
        commands(end+1)=string(cmd);[rc,text]=system(cmd);log=log+string(text);write(fullfile(outputRoot,'BUILD_LOG.txt'),log);
        assert(rc==0,'%s',text);
    end
    function check(n,p),checks(end+1)=struct('name',n,'pass',logical(p));end
end
function closeOwner(),try,r=rfly_original_getter_mex('close'); %#ok<NASGU>
catch,end,end
function ok=reject(f),ok=false;try,r=f(); %#ok<NASGU>
catch,ok=true;end,end
function write(p,t),f=fopen(p,'w','n','UTF-8');assert(f>=0);c=onCleanup(@()fclose(f));fprintf(f,'%s\n',t);end %#ok<NASGU>
function h=sha(p),f=fopen(p,'rb');assert(f>=0);c=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');m=java.security.MessageDigest.getInstance('SHA-256');m.update(typecast(b,'int8'));h=string(upper(reshape(dec2hex(typecast(m.digest(),'uint8'),2).',1,[])));end %#ok<NASGU>
