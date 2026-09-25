classdef OuterMailboxWorker < handle
    % Single-flight bidirectional mailbox for one persistent solver worker.
    properties (SetAccess=private)
        Submitted=0
        BusyRejected=0
        Accepted=0
        Expired=0
        WrongGeneration=0
        WorkerErrors=0
        MaximumInFlight=0
        LastCompletedDiagnostic=struct()
        LastErrorDiagnostic=''
        SendMaximumS=0
        PollMaximumS=0
        StopWasClean=false
    end
    properties (Access=private)
        InputQueue
        OutputQueue
        Future
        DeadlineNs
        ObservationAdmissionNs
        ObservationResultNs
        SubmittedNs=0
        SnapshotNs=0
        Generation=-1
        ExpiryCounted=false
        InFlight=false
        Closed=false
    end
    methods
        function obj=OuterMailboxWorker(fixedConstant,deadlineSeconds,observationAgeSeconds)
            assert(isa(fixedConstant,'parallel.pool.Constant'));
            assert(isscalar(deadlineSeconds) && isfinite(deadlineSeconds) && deadlineSeconds>0);
            obj.InputQueue=parallel.pool.PollableDataQueue(Destination='any');
            obj.OutputQueue=parallel.pool.PollableDataQueue;
            obj.DeadlineNs=deadlineSeconds*1e9;
            % Separate matched-observation age from queue, solve and return duration.
            % Preserve the command expiry and source timestamp.
            if nargin<3||isempty(observationAgeSeconds)
                obj.ObservationAdmissionNs=obj.DeadlineNs;
                obj.ObservationResultNs=obj.DeadlineNs;
            else
                assert(isscalar(observationAgeSeconds)&&isfinite(observationAgeSeconds)&&observationAgeSeconds>0);
                obj.ObservationAdmissionNs=observationAgeSeconds*1e9;
                obj.ObservationResultNs=obj.ObservationAdmissionNs+obj.DeadlineNs;
            end
            obj.Future=parfeval(backgroundPool,@gpenmpcNative.outerMailboxLoop,1, ...
                obj.InputQueue,obj.OutputQueue,fixedConstant);
        end
        function accepted=submit(obj,snapshot,generation,nowNs)
            accepted=false;
            if obj.InFlight,obj.BusyRejected=obj.BusyRejected+1;return;end
            obj.checkFuture();
            assert(~obj.Closed && ~strcmp(obj.Future.State,'finished'));
            assert(isscalar(generation) && isfinite(generation) && generation>=0);
            assert(isfield(snapshot,'snapshot_timestamp_ns') ...
                && isfinite(snapshot.snapshot_timestamp_ns) ...
                && snapshot.snapshot_timestamp_ns<=nowNs ...
                && nowNs-snapshot.snapshot_timestamp_ns<=obj.ObservationAdmissionNs);
            obj.SubmittedNs=nowNs;obj.SnapshotNs=snapshot.snapshot_timestamp_ns;
            obj.Generation=generation;obj.ExpiryCounted=false;
            timer=tic;send(obj.InputQueue,struct('kind','SOLVE', ...
                'generation',generation,'snapshot',snapshot));
            obj.SendMaximumS=max(obj.SendMaximumS,toc(timer));
            obj.InFlight=true;obj.Submitted=obj.Submitted+1;
            obj.MaximumInFlight=1;accepted=true;
        end
        function [available,result,reason]=poll(obj,nowNs,currentGeneration)
            timer=tic;available=false;result=struct;reason='IDLE';
            obj.checkFuture();
            if ~obj.InFlight,obj.PollMaximumS=max(obj.PollMaximumS,toc(timer));return;end
            expired=nowNs<obj.SubmittedNs || nowNs<obj.SnapshotNs ...
                || nowNs-obj.SubmittedNs>obj.DeadlineNs ...
                || nowNs-obj.SnapshotNs>obj.ObservationResultNs;
            if expired && ~obj.ExpiryCounted
                obj.Expired=obj.Expired+1;obj.ExpiryCounted=true;
            end
            response=poll(obj.OutputQueue,0);
            if isempty(response)
                if expired,reason='EXPIRED_STILL_BUSY_NO_REQUEUE';else,reason='PENDING';end
                obj.PollMaximumS=max(obj.PollMaximumS,toc(timer));return
            end
            obj.InFlight=false;
            if response.generation~=obj.Generation
                obj.WorkerErrors=obj.WorkerErrors+1;
                obj.LastErrorDiagnostic='Mailbox response generation differs from outstanding request.';
                reason='WORKER_ERROR_NO_COMMAND';
            elseif ~response.ok
                obj.WorkerErrors=obj.WorkerErrors+1;
                obj.LastErrorDiagnostic=response.error;reason='WORKER_ERROR_NO_COMMAND';
            else
                obj.LastCompletedDiagnostic=response.result;
                if expired
                    reason='LATE_RESULT_DISCARDED';
                elseif currentGeneration~=obj.Generation
                    obj.WrongGeneration=obj.WrongGeneration+1;
                    reason='STALE_GENERATION_DISCARDED';
                else
                    result=response.result;obj.Accepted=obj.Accepted+1;
                    available=true;reason='FRESH_RESULT';
                end
            end
            obj.PollMaximumS=max(obj.PollMaximumS,toc(timer));
        end
        function value=isBusy(obj),value=obj.InFlight;end
        function close(obj)
            if obj.Closed,return;end
            obj.Closed=true;
            try,send(obj.InputQueue,struct('kind','STOP'));catch,end
            timer=tic;
            while ~strcmp(obj.Future.State,'finished') && toc(timer)<5,pause(0.005);end
            if strcmp(obj.Future.State,'finished') && isempty(obj.Future.Error)
                summary=fetchOutputs(obj.Future);
                obj.StopWasClean=logical(summary.clean_stop);
            else
                cancel(obj.Future);
            end
            close(obj.InputQueue);close(obj.OutputQueue);
        end
        function delete(obj),obj.close();end
    end
    methods (Access=private)
        function checkFuture(obj)
            if strcmp(obj.Future.State,'finished') && ~obj.Closed
                obj.WorkerErrors=obj.WorkerErrors+1;
                if isempty(obj.Future.Error)
                    obj.LastErrorDiagnostic='Persistent worker exited unexpectedly.';
                else
                    obj.LastErrorDiagnostic=getReport(obj.Future.Error,'extended','hyperlinks','off');
                end
            end
        end
    end
end
