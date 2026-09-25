classdef RflyLocalPhaseView < handle
    % Mirror reference phase from committed RLC records.
    % Initialize phase to zero before the first commit.
    properties (SetAccess=private)
        Failed=false
        CommitCount=uint64(0)
    end
    properties (Access=private)
        Bundle
        Trajectory
        Registered
        Association
        ConfigurationBytes
        ReferenceBytes
        Last=[]
        LastRecord=[]
        LastView=[]
        Lifecycle='PREPARED_PAUSED'
    end
    methods
        function obj=RflyLocalPhaseView(bundle,trajectory,registered,association)
            assert(isstruct(association)&&association.local_full_inner ...
                &&strcmp(association.registration_result,'Registered') ...
                &&strcmp(association.echo_confirmation_result,'Confirmed') ...
                &&strcmpi(association.execution_session_sha256,registered.execution_session_sha256), ...
                'gpenmpcNative:LocalPhaseRegistration');
            e=association.echo;p=association.confirm_receipt.parsed_fields;
            assert(e.uid==registered.identity.uid&&e.process_session_generation==registered.identity.boot_generation ...
                &&e.system==registered.identity.system&&e.component==registered.identity.component ...
                &&strcmpi(e.configuration_payload_sha256,registered.configuration_sha256) ...
                &&p.leg==registered.leg_index&&strcmpi(p.task_sha,registered.task_sha256) ...
                &&p.state==3&&p.start_requests==0&&p.stop_requests==0&&p.session_fault==0, ...
                'gpenmpcNative:LocalPhaseRegistration');
            obj.Bundle=bundle;obj.Trajectory=trajectory;obj.Registered=registered;obj.Association=association;
            obj.ConfigurationBytes=uint8(sscanf(char(registered.configuration_sha256),'%2x'));
            obj.ReferenceBytes=uint8(sscanf(char(registered.reference_asset_sha256),'%2x'));
            obj.view(); % Check original shared trajectory/leg before runtime.
        end
        function ingest(obj,record)
            assert(~obj.Failed,'gpenmpcNative:LocalPhaseFailed');
            try
                assert(~strcmp(obj.Lifecycle,'NATIVE_LAND')&&~strcmp(obj.Lifecycle,'GROUND'),'gpenmpcNative:LocalPhaseSuspended');
                c=gpenmpcNative.RflyLocalCommittedDecoder(record.message,record.original_host_receive_ns);r=obj.Registered;
                assert(c.learning_audit_available&&isequal(c.identity,r.identity)&&c.leg_index==r.leg_index ...
                    &&isequal(c.configuration_sha256,obj.ConfigurationBytes) ...
                    &&isequal(c.reference_asset_sha256,obj.ReferenceBytes) ...
                    &&record.original_host_receive_ns>=obj.Association.original_host_receive_ns ...
                    &&c.installed_phase2(1)>=0&&c.installed_phase2(1)<=obj.Trajectory.total_duration_s, ...
                    'gpenmpcNative:LocalPhaseCommit');
                if ~isempty(obj.Last)
                    a=obj.Last;
                    assert(c.source_generation>a.source_generation&&c.output_generation>a.output_generation ...
                        &&c.joint_installs>a.joint_installs&&c.source_timestamp_ns>a.source_timestamp_ns ...
                        &&c.installed_phase2(1)>=a.installed_phase2(1) ...
                        &&record.original_host_receive_ns>=obj.LastRecord.original_host_receive_ns, ...
                        'gpenmpcNative:LocalPhaseOrder');
                end
                obj.Last=c;obj.LastRecord=record;obj.CommitCount=c.joint_installs;obj.Lifecycle='FLIGHT';obj.LastView=[];
            catch ex,obj.Failed=true;rethrow(ex);end
        end
        function suspend(obj,lifecycle)
            assert(~obj.Failed&&ismember(lifecycle,{'NATIVE_LAND','GROUND'}),'gpenmpcNative:LocalPhaseSuspension');
            obj.Lifecycle=lifecycle;obj.LastView=[]; % Update the lifecycle view.
        end
        function v=view(obj)
            assert(~obj.Failed,'gpenmpcNative:LocalPhaseFailed');
            % Reuse the immutable leg view between validated commits.
            if ~isempty(obj.LastView),v=obj.LastView;return;end
            q=0;if ~isempty(obj.Last),q=obj.Last.installed_phase2(1);end
            k=double(obj.Registered.leg_index);
            s=struct('leg_index',k,'phase_s',q,'payload_kg',obj.Bundle.legs{k}.meta.payload_kg, ...
                'outer_suspended',ismember(obj.Lifecycle,{'NATIVE_LAND','GROUND'}));
            v=gpenmpcNative.rflyCanonicalDeliveryTimeView(obj.Bundle, ...
                struct('service',s,'state',obj.Lifecycle,'failed',false),obj.Trajectory);
            v.owner='ACTUAL_JOINT_INSTALLED_RLC2_OR_EXPLICIT_REGISTERED_LEG_INITIALIZER';
            v.board_commit_count=obj.CommitCount;v.initializer_only=isempty(obj.Last);
            obj.LastView=v;
        end
        function [p,v]=bindSource(obj,source)
            v=obj.view();s=gpenmpcNative.RflyLocalSnapshotDecoder(source.message,source.original_host_receive_ns);
            assert(isequal(s.identity,obj.Registered.identity) ...
                &&source.original_host_receive_ns>=obj.Association.original_host_receive_ns, ...
                'gpenmpcNative:LocalPhaseSource');
            if isempty(obj.Last)
                % Raw SERIAL_CONTROL receipt is retained by the association.
                raw=obj.Association.confirm_receipt.original_session_line_bytes(:);
                generation=uint64(0);origin='REGISTERED_LEG_INITIALIZER';
            else
                assert(obj.Last.source_generation<=s.source_generation ...
                    &&obj.Last.source_timestamp_ns<=s.original_sample_us*uint64(1000), ...
                    'gpenmpcNative:LocalPhaseFutureCommit');
                raw=obj.Last.original_bytes;generation=obj.Last.source_generation;origin='PREVIOUS_OR_SAME_JOINT_INSTALLED_RLC2';
            end
            p=struct('source_rls_sha256',digest(source.message),'saved_task_time_s',v.saved_task_time_s, ...
                'leg_index',v.global_leg_index,'original_bytes',raw,'origin',origin, ...
                'origin_commit_generation',generation,'host_phase_advanced',false,'plant_truth_used',false);
        end
    end
end
function s=hex(b),s=upper(reshape(dec2hex(b,2).',1,[]));end
function h=digest(b),m=java.security.MessageDigest.getInstance('SHA-256');m.update(typecast(b(:),'int8'));h=reshape(typecast(m.digest(),'uint8'),[],1);end
