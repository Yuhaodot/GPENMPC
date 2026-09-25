function report=test_short_environment_prebind_transition(outputRoot)
% Replay the startup pre-bind race with retained records.
arguments,outputRoot (1,1) string,end
root=string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'host_runtime'),fullfile(root,'matlab_validation'));
assert(~isfolder(outputRoot)&&~isfile(outputRoot), ...
    'gpenmpcShort:OutputExists','Preserve prior evidence.');
mkdir(outputRoot);diary(fullfile(outputRoot,'MATLAB_DIARY.txt'));
checks=struct('name',{},'pass',{});liveRoot=fullfile(gpenmpc_external_path('environment_prebind'),'SHORT_HIL');
rawPath=fullfile(liveRoot,'RAW_BOARD_LOCAL_SHORT_HIL.mat');
s=load(rawPath,'cfg','rawIo');p=s.cfg.delivery_environment_contract;
% Replay six diagnostics omitted by the final observation drain.
% The first service event is raw item 298; the first ENV send falls between items 300 and 302.
preTx=[298 300];postTx=[302 304 306 308];boundIndex=310;
initial=rxEvent(s.rawIo.raw_truth_datagrams{preTx(1)});
bound=rxEvent(s.rawIo.raw_truth_datagrams{boundIndex});
q=p;q.allow_unbound_pre_session=true;
d0=m600check.decodeCopterSimDeliveryDiagnostics(initial.bytes,1,NaN,q);
d1=m600check.decodeCopterSimDeliveryDiagnostics(bound.bytes,1,NaN,q);
check('initial_and_bound_packets_found',d0.packet_valid&& ...
    d0.environment_extension.initial_not_applied&&~d0.environment_extension.session_bound&& ...
    d1.packet_valid&&d1.environment_extension.session_bound&&d1.environment_extension.mass_ack_valid);
tx=s.rawIo.raw_environment_transmit_datagrams{1};
sendEvent=struct('kind','ENV_TX','bytes',tx.bytes(:), ...
    'original_host_send_ns',tx.original_host_send_ns,'send_returned',true);
ledger=gpenmpcNative.RflyLocalEnvironmentLedger(p,1,16);
for k=preTx,ledger.received(rxEvent(s.rawIo.raw_truth_datagrams{k}));end
st=ledger.status();
check('initial_diagnostics_before_send_are_observation_only',~st.failed&&st.prebind_observed==2&& ...
    st.received==0&&~st.bound_observed&&isempty(ledger.resolve(tx.frame(3))));
ledger.sent(sendEvent);
for k=postTx,ledger.received(rxEvent(s.rawIo.raw_truth_datagrams{k}));end
st=ledger.status();
check('initial_diagnostics_after_first_send_still_no_ack',st.prebind_observed==6&& ...
    st.received==0&&isempty(ledger.resolve(tx.frame(3))));
ledger.received(bound);st=ledger.status();resolved=ledger.resolve(tx.frame(3));
check('first_bound_diagnostic_is_retained_without_retroactive_ack',st.bound_observed&& ...
    st.received==1&&isempty(resolved));
returned=initial;returned.original_host_receive_ns=bound.original_host_receive_ns+uint64(1);
id='';try,ledger.received(returned);catch ex,id=ex.identifier;end
check('unbound_return_after_binding_is_fail_closed',strcmp(id,'gpenmpcNative:EnvironmentLedgerUnboundAfterBind')&&ledger.Failed);
badLedger=gpenmpcNative.RflyLocalEnvironmentLedger(p,1,16);bad=initial;bad.bytes(1)=bitxor(bad.bytes(1),uint8(1));
id='';try,badLedger.received(bad);catch ex,id=ex.identifier;end
check('malformed_packet_keeps_primary_identifier',strcmp(id,'gpenmpcNative:EnvironmentLedgerDiagnosticPacket')&&badLedger.Failed);
report=struct('schema','ENVIRONMENT_PREBIND_TRANSITION_REPLAY_V1', ...
    'passed',all([checks.pass]),'test_count',numel(checks),'tests_passed',sum([checks.pass]), ...
    'checks',checks,'source_raw_path',rawPath,'source_raw_sha256',fileSha(rawPath), ...
    'environment_ledger_source',which('gpenmpcNative.RflyLocalEnvironmentLedger'), ...
    'environment_ledger_sha256',fileSha(which('gpenmpcNative.RflyLocalEnvironmentLedger')), ...
    'COM_open',0,'board_actions',0,'source_binding_granted_by_prebind',false, ...
    'control_authority_granted_by_prebind',false);
writeText(fullfile(outputRoot,'RESULT.json'),jsonencode(report,PrettyPrint=true));
save(fullfile(outputRoot,'RAW.mat'),'report','initial','bound','sendEvent','resolved');
diary off;disp(jsonencode(report));
    function check(name,pass)
        checks(end+1)=struct('name',name,'pass',logical(pass)); %#ok<AGROW>
        assert(pass,'gpenmpcShort:Check','%s',name);
    end
end
function event=rxEvent(raw)
assert(numel(raw.bytes)==264,'gpenmpcShort:RawIdentity','Expected an exact 264-byte delivery diagnostic.');
event=struct('kind','DIAGNOSTIC_RX','bytes',uint8(raw.bytes(:)), ...
    'original_host_receive_ns',raw.original_host_receive_ns);
end
function writeText(path,text)
f=fopen(path,'w');assert(f>=0,'gpenmpcShort:FileOpen','Cannot open %s.',path);
c=onCleanup(@()fclose(f));fprintf(f,'%s\n',text);
end
function h=fileSha(path)
md=java.security.MessageDigest.getInstance('SHA-256');f=fopen(path,'rb');
assert(f>=0,'gpenmpcShort:FileOpen','Cannot open %s.',path);c=onCleanup(@()fclose(f));
while true,b=fread(f,1024*1024,'*uint8');if isempty(b),break,end;md.update(b);end
h=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
