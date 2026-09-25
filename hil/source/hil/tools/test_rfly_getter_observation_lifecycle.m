function report=test_rfly_getter_observation_lifecycle()
% Test getter lifecycle with retained records.
build=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(build,'host_runtime'));
target=fullfile(build,'tools','GETTER_OBSERVATION_LIFECYCLE_HOST_ONLY.mat');
assert(~isfile(target),'gpenmpcHost:ExistingEvidence','Choose an unused output path.');
source=fullfile(gpenmpc_external_path('matlab_noui_original_reader'),'MATLAB_ORIGINAL_GETTERS.mat');
data=load(source,'last','records','reads');base=data.last;records=data.records;reads=data.reads;
dll='990850A2F40F3FCC2A6C47E63A4065B60FF49AA39CC4749FF443963B06F2EF7E';
checks=struct();raw=struct();
obj=gpenmpcNative.RflyLocalOriginalGetterBuffer(256,dll);
obj.observeOnly(batch(1:256));obj.observeOnly(batch(257:512));s=obj.status();raw.prestart=s;
checks.prestart_512_over_256_capacity_retains_no_source_candidate= ...
    s.accepted_getters==512&&s.observation_only_getters==512&&s.retained_getters==0&& ...
    s.explicitly_retired_getters==0&&s.matched_sources==0&&~s.source_matching_started&& ...
    ~s.failed&&~s.board_authority;
obj.ingest(batch(513:768));s=obj.status();raw.active=s;
assert(s.retained_getters==256&&s.source_matching_started&&s.accepted_getters==768);
id=rejection(@()obj.ingest(batch(769:770)));s=obj.status();raw.capacity=s;
checks.active_unretired_capacity_still_rejects_without_overwrite=strcmp(id,'gpenmpcNative:GetterCapacity')&& ...
    s.failed&&s.retained_getters==256&&s.accepted_getters==768&&s.observation_only_getters==512;
obj=gpenmpcNative.RflyLocalOriginalGetterBuffer(256,dll);obj.ingest(batch(1:10));
id=rejection(@()obj.observeOnly(batch(11:20)));s=obj.status();raw.active_discard=s;
checks.active_cannot_switch_to_discard=strcmp(id,'gpenmpcNative:GetterActiveObservation')&& ...
    s.failed&&s.retained_getters==10&&s.accepted_getters==10&&s.observation_only_getters==0;
obj=gpenmpcNative.RflyLocalOriginalGetterBuffer(256,dll);obj.ingest(batch(1:256));
obj.closeSourceMatching();obj.observeOnly(batch(257:512));obj.observeOnly(batch(513:768));
s=obj.status();raw.closed=s;
checks.safety_closed_observation_preserves_original_unretired=~s.failed&&s.source_matching_closed&& ...
    s.retained_getters==256&&s.explicitly_retired_getters==0&&s.observation_only_getters==512&& ...
    s.accepted_getters==768&&s.matched_sources==0&&~s.board_authority;
id=rejection(@()obj.bind([],uint64(0),[],[],[]));
checks.safety_closed_cannot_bind=strcmp(id,'gpenmpcNative:GetterMatchingInactive');
obj=gpenmpcNative.RflyLocalOriginalGetterBuffer(256,dll);obj.observeOnly(batch(1:256));
id=rejection(@()obj.bind([],uint64(0),[],[],[]));
checks.prestart_observation_cannot_bind=strcmp(id,'gpenmpcNative:GetterMatchingInactive');
report=struct('scope','HOST_ONLY_ACTUAL_RETAINED_GETTER_LIFECYCLE','pass',all(structfun(@logical,checks)), ...
    'checks',checks,'capacity_unchanged',256,'source',source, ...
    'COM',0,'sockets',0,'CopterSim',0,'board',0,'model_runs',0,'board_authority',false);
save(target,'report','raw');disp(jsonencode(report));assert(report.pass);
    function b=batch(ix),b=base;b.records=records(:,ix);b.original_read_ns=reads(ix);end
end
function id=rejection(fn)
id='';try,fn();catch ex,id=ex.identifier;end
end
