classdef RflyLocalTaskAsset < handle
    % Load and byte-verify immutable task data once per host run.
    properties (SetAccess=private)
        Path
        Sha256
        ConfigurationSha256
        FileBytes
        FileReads=uint64(0)
    end
    properties (Access=private)
        Reference
        ActualWind
        EstimatedWind
    end
    methods
        function obj=RflyLocalTaskAsset(taskSource)
            obj.Path=string(taskSource.path);
            obj.ConfigurationSha256=hash(taskSource.configuration_sha256);
            f=fopen(obj.Path,'rb');assert(f>=0,'gpenmpcNative:LocalTaskAssetFile');
            c=onCleanup(@()fclose(f));bytes=fread(f,Inf,'*uint8');clear c
            m=java.security.MessageDigest.getInstance('SHA-256');m.update(typecast(bytes,'int8'));
            obj.Sha256=reshape(typecast(m.digest(),'uint8'),[],1);obj.FileBytes=numel(bytes);
            assert(isequal(obj.Sha256,hash(taskSource.sha256)),'gpenmpcNative:LocalTaskAssetHash');
            t=load(obj.Path,'physicalTask');t=t.physicalTask;
            assert(strcmp(t.execution_method_binding.architecture,'A1_COORDINATED_PHYSICAL') ...
                &&isequal(hash(t.execution_method_binding.effective_configuration_payload_sha256),obj.ConfigurationSha256), ...
                'gpenmpcNative:LocalTaskAssetMethod');
            obj.Reference=t.reference;obj.FileReads=uint64(1);
            r=obj.Reference;
            time=double(r.global_time_s(:));
            assert(numel(time)>1&&all(isfinite(time))&&all(diff(time)>0) ...
                &&isequal(size(r.actual_wind_xy_mps),[numel(time),2]) ...
                &&isequal(size(r.wind_estimate_xy_mps),[numel(time),2]) ...
                &&all(isfinite(r.wind_estimate_xy_mps(:))) ...
                &&all(isfinite(r.actual_wind_xy_mps(:))),'gpenmpcNative:LocalTaskAssetReference');
            % interp1's matrix-valued linear path uses this same rectangular
            % grid. Construct it once for the immutable asset, not per beat.
            obj.ActualWind=griddedInterpolant({time,(1:2)'},double(r.actual_wind_xy_mps),'linear','linear');
            obj.EstimatedWind=griddedInterpolant({time,(1:2)'},double(r.wind_estimate_xy_mps),'linear','linear');
        end
        function [r,h]=read(obj,taskSource)
            assert(string(taskSource.path)==obj.Path&&isequal(hash(taskSource.sha256),obj.Sha256) ...
                &&isequal(hash(taskSource.configuration_sha256),obj.ConfigurationSha256), ...
                'gpenmpcNative:LocalTaskAssetBinding','Cached task cannot change mid-session.');
            r=obj.Reference;h=obj.Sha256;
        end
        function [actual,estimate,selection]=windAt(obj,globalTime,legIndex)
            assert(isnumeric(globalTime)&&isreal(globalTime)&&isscalar(globalTime)&&isfinite(globalTime), ...
                'gpenmpcNative:CanonicalTaskWind');
            time=obj.Reference.global_time_s;
            query=min(max(globalTime,time(1)),time(end));
            lo=1;hi=numel(time);
            while lo<hi
                mid=floor((lo+hi+1)/2);if time(mid)<=query,lo=mid;else,hi=mid-1;end
            end
            row=lo;assert(obj.Reference.leg_index(row)==legIndex,'gpenmpcNative:CanonicalTaskWindLeg');
            actual=obj.ActualWind({double(query),(1:2)'}).';
            estimate=obj.EstimatedWind({double(query),(1:2)'}).';
            selection=struct('schema','RFLY_CANONICAL_LINEAR_TASK_WIND_V1','saved_task_time_s',globalTime, ...
                'evaluated_task_time_s',query,'left_source_row',row,'right_source_row',min(row+1,numel(time)), ...
                'leg_index',legIndex,'actual_wind_xy_mps',actual,'wind_estimate_xy_mps',estimate, ...
                'arithmetic_source','CANONICAL_WHOLE_TASK_EXOGENOUS_AT_LINEAR','online_wind_measurement',false);
        end
    end
end
function h=hash(x)
if isa(x,'uint8'),h=x(:);else,h=uint8(sscanf(char(x),'%2x'));end
assert(numel(h)==32);
end
