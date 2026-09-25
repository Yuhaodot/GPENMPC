classdef LiveHilWireLoop < handle
    % LIVEHILWIRELOOP Own the 100 Hz HIL transport and display loop.
    % The tick() input is the plant state; RflySim3D datagrams are display-only.
    % Construction is inert; the caller opens MavlinkSerialLink explicitly.

    properties (SetAccess=private)
        Link
        Board
        Poller
        TickIndex (1,1) uint64 = uint64(0)
        WireTimeS (1,1) double = 0
        NextWallNs (1,1) double = 0
        MaximumWallLatenessS (1,1) double = 0
        HilSensorCount (1,1) uint64 = uint64(0)
        HilGpsCount (1,1) uint64 = uint64(0)
        HilStateCount (1,1) uint64 = uint64(0)
        SetpointCount (1,1) uint64 = uint64(0)
        SegmentChunksSent (1,1) uint64 = uint64(0)
        SegmentSequence (1,1) uint32 = uint32(2000)
        VisualCount (1,1) uint64 = uint64(0)
        VisualSequence (1,1) uint64 = uint64(0)
        InitialPayloadKg (1,1) double = 2.21
    end

    properties (Access=private)
        PendingSegment (1,:) uint8 = uint8([])
        PendingSegmentDueTick (1,1) uint64 = uint64(0)
        VisualPort = []
        Closed (1,1) logical = false
    end

    properties (Constant)
        DtS = 0.01
        GpsPeriodTicks = 10
        SetpointPeriodTicks = 2
        SegmentPeriodTicks = 20
        HeartbeatPeriodTicks = 100
        ManualPeriodTicks = 5
        VisualPeriodTicks = 3
        VisualHost = "127.0.0.1"
        VisualPortNumber = 28430
        TaskOriginNedM = [-34.518;-75.710;0]
        HeartbeatWatchdogNs = 2500000000
        RunId = "GPENMPC_LIVE_DELIVERY"
    end

    methods
        function obj=LiveHilWireLoop(link,board,poller,initialPayloadKg)
            arguments
                link (1,1) gpenmpcNative.MavlinkSerialLink
                board (1,1) gpenmpcNative.LiveBoardState
                poller (1,1) gpenmpcNative.Se3StatusPoller
                initialPayloadKg (1,1) double {mustBeFinite,mustBeNonnegative}=2.21
            end
            assert(link.isOpen(),'gpenmpcNative:LiveWireClosedLink', ...
                'The explicit MATLAB serial owner must already be open.');
            obj.Link=link;obj.Board=board;obj.Poller=poller;
            obj.InitialPayloadKg=initialPayloadKg;
            obj.VisualPort=udpport("datagram","IPV4");
            obj.NextWallNs=obj.monotonicNs();
        end

        function nowNs=tick(obj,snapshot,referenceGlobalNed,phase,allowReference)
            arguments
                obj
                snapshot (1,1) struct
                referenceGlobalNed=[]
                phase=[]
                allowReference (1,1) logical=false
            end
            obj.requireUsable();
            obj.waitForTickBoundary();
            boundaryNs=obj.monotonicNs();
            obj.MaximumWallLatenessS=max(obj.MaximumWallLatenessS, ...
                max(0,(boundaryNs-obj.NextWallNs)/1e9));

            counts=obj.Link.sendHilSensors(obj.WireTimeS,snapshot, ...
                obj.Board.Armed,double(obj.HilSensorCount), ...
                mod(double(obj.TickIndex),obj.GpsPeriodTicks)==0);
            obj.HilSensorCount=obj.HilSensorCount+uint64(counts.hil_sensor);
            obj.HilGpsCount=obj.HilGpsCount+uint64(counts.hil_gps);
            obj.HilStateCount=obj.HilStateCount+uint64(counts.hil_state);

            if mod(double(obj.TickIndex),obj.ManualPeriodTicks)==0
                obj.Link.sendNeutralManualControl();
            end
            if mod(double(obj.TickIndex),obj.HeartbeatPeriodTicks)==0
                obj.Link.sendHeartbeat();
            end

            reference=[];
            if allowReference
                assert(isstruct(referenceGlobalNed)&&isscalar(referenceGlobalNed), ...
                    'gpenmpcNative:LiveWireReferenceMissing', ...
                    'Reference publication was admitted without a reference.');
                reference=obj.toLocalReference(referenceGlobalNed);
                if mod(double(obj.TickIndex),obj.SetpointPeriodTicks)==0
                    obj.Link.sendSetpoint(round(obj.WireTimeS*1000), ...
                        reference.position_ned_m,reference.velocity_ned_mps, ...
                        reference.acceleration_ned_mps2,reference.yaw_rad);
                    obj.SetpointCount=obj.SetpointCount+uint64(1);
                end
                obj.sendSegmentIfDue(reference,phase);
            elseif ~isempty(obj.PendingSegment)
                error('gpenmpcNative:LiveWirePendingSegmentAtAuthorityStop', ...
                    'Reference authority stopped with a partial segment pending.');
            end

            if mod(double(obj.TickIndex),obj.VisualPeriodTicks)==0
                obj.sendVisual(snapshot,referenceGlobalNed,phase,boundaryNs);
            end
            obj.dispatch();
            obj.Poller.tick(obj.Link,obj.monotonicNs());
            obj.dispatch();
            % Return a timestamp taken after all receive dispatch.  Causal
            % consumers can therefore require rx_ns <= nowNs without the
            % subtle pre-I/O timestamp inversion present in older wrappers.
            nowNs=obj.monotonicNs();
            if obj.Board.LastHeartbeatNs>0 ...
                    &&nowNs-obj.Board.LastHeartbeatNs>obj.HeartbeatWatchdogNs
                error('gpenmpcNative:LiveWireHeartbeatWatchdog', ...
                    'BOARD_HEARTBEAT_WATCHDOG_TIMEOUT');
            end

            obj.TickIndex=obj.TickIndex+uint64(1);
            obj.WireTimeS=obj.WireTimeS+obj.DtS;
            obj.NextWallNs=obj.NextWallNs+round(obj.DtS*1e9);
        end

        function count=dispatch(obj)
            obj.requireUsable();
            messages=obj.Link.drain();count=numel(messages);
            nowNs=obj.monotonicNs();
            for k=1:count
                obj.Board.accept(messages{k},nowNs,obj.Poller);
            end
        end

        function value=status(obj)
            value=struct( ...
                'schema','GPENMPC_MATLAB_NATIVE_LIVE_HIL_WIRE_LOOP_V1', ...
                'tick_index',double(obj.TickIndex), ...
                'wire_time_s',obj.WireTimeS, ...
                'maximum_wall_lateness_s',obj.MaximumWallLatenessS, ...
                'hil_sensor_count',double(obj.HilSensorCount), ...
                'hil_gps_count',double(obj.HilGpsCount), ...
                'hil_state_count',double(obj.HilStateCount), ...
                'setpoint_count',double(obj.SetpointCount), ...
                'segment_chunks_sent',double(obj.SegmentChunksSent), ...
                'visual_datagrams_sent',double(obj.VisualCount), ...
                'pending_segment',~isempty(obj.PendingSegment), ...
                'matlab_formal_runtime',true, ...
                'python_in_formal_control_or_plant',false, ...
                'rflysim_role','DISPLAY_ONLY', ...
                'closed',obj.Closed);
        end

        function close(obj)
            if obj.Closed,return,end
            port=obj.VisualPort;obj.VisualPort=[];
            if ~isempty(port)
                try,delete(port);catch,end
            end
            obj.Closed=true;
        end

        function delete(obj)
            obj.close();
        end
    end

    methods (Access=private)
        function waitForTickBoundary(obj)
            while true
                remaining=(obj.NextWallNs-obj.monotonicNs())/1e9;
                if remaining<=0,break,end
                if remaining>0.0005
                    pause(max(remaining-0.00035,0));
                end
            end
        end

        function sendSegmentIfDue(obj,reference,phase)
            if ~isempty(obj.PendingSegment) ...
                    &&obj.TickIndex>=obj.PendingSegmentDueTick
                obj.Link.sendSegmentChunk(obj.PendingSegment);
                obj.SegmentChunksSent=obj.SegmentChunksSent+uint64(1);
                obj.PendingSegment=uint8([]);
            end
            if mod(double(obj.TickIndex),obj.SegmentPeriodTicks)~=0,return,end
            assert(isempty(obj.PendingSegment), ...
                'gpenmpcNative:LiveWireSegmentOverlap','SEGMENT_SEQUENCE_OVERLAP');
            obj.SegmentSequence=obj.SegmentSequence+uint32(1);
            wind=zeros(3,1);payload=obj.InitialPayloadKg;
            if isstruct(phase)&&isscalar(phase)
                if isfield(phase,'wind_estimate_xy_mps')
                    xy=double(phase.wind_estimate_xy_mps(:));
                    assert(numel(xy)==2&&all(isfinite(xy)), ...
                        'gpenmpcNative:LiveWireWind','Invalid phase wind estimate.');
                    wind(1:2)=xy;
                end
                if isfield(phase,'payload_kg')
                    payload=double(phase.payload_kg);
                end
            end
            chunks=obj.Link.segmentPayload(obj.SegmentSequence, ...
                reference.position_ned_m,reference.yaw_rad,wind,payload, ...
                obj.InitialPayloadKg);
            obj.Link.sendSegmentChunk(chunks{1});
            obj.SegmentChunksSent=obj.SegmentChunksSent+uint64(1);
            obj.PendingSegment=chunks{2};
            obj.PendingSegmentDueTick=obj.TickIndex+uint64(2);
        end

        function sendVisual(obj,snapshot,referenceGlobalNed,phase,nowNs)
            software=double(snapshot.rotor_thrust_software_order_n(:));
            assert(numel(software)==6&&all(isfinite(software))&&all(software>=0), ...
                'gpenmpcNative:LiveWireVisualRotor','Invalid display rotor state.');
            px4=zeros(6,1);px4([5;1;4;6;2;3])=software;
            payload=struct( ...
                'schema','GPENMPC_VISUAL_STATE_V1', ...
                'run_id',obj.RunId,'seq',double(obj.VisualSequence), ...
                'sim_time_s',double(snapshot.sim_time_s), ...
                'host_monotonic_ns',double(nowNs), ...
                'layer','HARDWARE_CLOSED_LOOP', ...
                'position_ned_m',double(snapshot.position_ned_m(:)).', ...
                'velocity_ned_mps',double(snapshot.velocity_ned_mps(:)).', ...
                'quaternion_wxyz_body_to_ned', ...
                    double(snapshot.quaternion_wxyz_body_to_ned(:)).', ...
                'omega_frd_rad_s',double(snapshot.body_rate_frd_rad_s(:)).', ...
                'rotor_thrust_n',px4.', ...
                'valid',true, ...
                'control_mode','A1_COORDINATED_PHYSICAL__RA_CTRL_MODE_1', ...
                'payload_kg',obj.phasePayload(phase), ...
                'ground_contact',logical(snapshot.contact_active));
            if isstruct(referenceGlobalNed)&&isscalar(referenceGlobalNed) ...
                    &&isfield(referenceGlobalNed,'position_ned_m')
                payload.reference_position_ned_m= ...
                    double(referenceGlobalNed.position_ned_m(:)).';
            end
            encoded=unicode2native(jsonencode(payload),'UTF-8');
            assert(numel(encoded)<=8192, ...
                'gpenmpcNative:LiveWireVisualOversize','VISUAL_DATAGRAM_OVERSIZE');
            write(obj.VisualPort,uint8(encoded),"uint8", ...
                obj.VisualHost,obj.VisualPortNumber);
            obj.VisualSequence=obj.VisualSequence+uint64(1);
            obj.VisualCount=obj.VisualCount+uint64(1);
        end

        function reference=toLocalReference(obj,globalReference)
            required={'position_ned_m','velocity_ned_mps', ...
                'acceleration_ned_mps2','yaw_rad'};
            assert(all(isfield(globalReference,required)), ...
                'gpenmpcNative:LiveWireReferenceSchema','Reference fields missing.');
            reference=globalReference;
            reference.position_ned_m=double(globalReference.position_ned_m(:)) ...
                -obj.TaskOriginNedM;
            reference.velocity_ned_mps=double(globalReference.velocity_ned_mps(:));
            reference.acceleration_ned_mps2= ...
                double(globalReference.acceleration_ned_mps2(:));
            reference.yaw_rad=double(globalReference.yaw_rad);
            assert(all(isfinite([reference.position_ned_m; ...
                reference.velocity_ned_mps;reference.acceleration_ned_mps2; ...
                reference.yaw_rad])), ...
                'gpenmpcNative:LiveWireReferenceFinite','Reference is nonfinite.');
        end

        function value=phasePayload(~,phase)
            value=[];
            if isstruct(phase)&&isscalar(phase)&&isfield(phase,'payload_kg')
                candidate=double(phase.payload_kg);
                if isfinite(candidate)&&candidate>=0,value=candidate;end
            end
        end

        function requireUsable(obj)
            assert(~obj.Closed&&obj.Link.isOpen(), ...
                'gpenmpcNative:LiveWireUnavailable','Live wire loop is unavailable.');
        end
    end

    methods (Static)
        function value=monotonicNs()
            value=double(javaMethod('nanoTime','java.lang.System'));
        end
    end
end
