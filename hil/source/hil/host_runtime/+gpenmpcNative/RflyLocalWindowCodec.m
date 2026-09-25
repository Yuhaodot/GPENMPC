classdef RflyLocalWindowCodec
    % Encode RWW1 with little-endian binary64 arrays, preserving unused NaNs.
    methods (Static)
        function [bytes,packets,identity]=encode(window,registered,serviceIo)
            if nargin<3,serviceIo=@() [];end
            assert(isa(serviceIo,'function_handle'));serviceIo();
            w=window;e=registered;
            assert(isstruct(w)&&isscalar(w)&&isstruct(e)&&isscalar(e));
            required={'uid','session_generation','source_system','source_component', ...
                'execution_session_sha256','task_sha256','configuration_sha256','reference_asset_sha256','leg_index'};
            assert(all(isfield(e,required))&&isa(e.uid,'uint64')&&isscalar(e.uid)&&e.uid>0 ...
                &&isa(e.session_generation,'uint64')&&isscalar(e.session_generation)&&e.session_generation>0 ...
                &&isa(e.source_system,'uint8')&&isscalar(e.source_system)&&e.source_system>0 ...
                &&isa(e.source_component,'uint8')&&isscalar(e.source_component)&&e.source_component>0 ...
                &&isscalar(e.leg_index)&&e.leg_index>=1&&e.leg_index<=5&&e.leg_index==fix(e.leg_index), ...
                'gpenmpcNative:LocalWindowIdentity','Explicit registered owner identity required.');
            assert(w.schema==1&&w.capacity==256&&w.leg_index==e.leg_index ...
                &&isa(w.window_generation,'uint64')&&isscalar(w.window_generation)&&w.window_generation>0 ...
                &&isa(w.reference_asset_sha256,'uint8')&&isequal(w.reference_asset_sha256(:),hashBytes(e.reference_asset_sha256)), ...
                'gpenmpcNative:LocalWindowBinding','Prepared window must match this registered reference/leg.');
            fields={'schema','capacity','leg_index','source_first_row','source_total_rows','row_count','binding_mode'};
            ints=zeros(7,1,'uint32');
            for k=1:7
                v=w.(fields{k});assert(isscalar(v)&&isnumeric(v)&&isfinite(double(v))&&v>=0 ...
                    &&double(v)<=double(intmax('uint32'))&&double(v)==fix(double(v)));ints(k)=uint32(v);
            end
            assert(w.source_first_row>=1&&w.row_count>=2&&w.row_count<=256 ...
                &&double(w.source_first_row)+double(w.row_count)-1<=double(w.source_total_rows) ...
                &&ismember(w.binding_mode,[1 2]),'gpenmpcNative:LocalWindowRows','No invented or truncated source rows.');
            scalarFields={'nominal_duration_s','total_duration_s','prefix_duration_s','relaunch_duration_s','vertical_frame_offset_ned_m'};
            scalars=zeros(5,1);
            for k=1:5,v=w.(scalarFields{k});assert(isa(v,'double')&&isreal(v)&&isscalar(v)&&isfinite(v));scalars(k)=v;end
            arrayFields={'time_s','nominal_jet','prefix_coefficients','ground_jet','rest_jet','relaunch_offset_ned_m'};
            shapes={[256 1],[256 3 4],[3 8 4 2],[3 4],[3 4],[3 1]};
            body=[littleBytes(ints);hashBytes(w.reference_asset_sha256);littleBytes(w.window_generation);littleBytes(scalars)];
            for k=1:6
                v=w.(arrayFields{k});assert(isa(v,'double')&&isreal(v)&&isequal(size(v),shapes{k}), ...
                    'gpenmpcNative:LocalWindowShape','Original fixed-capacity array shape required.');
                body=[body;littleBytes(v)]; %#ok<AGROW>
                serviceIo();
            end
            assert(numel(body)==28484);
            manifest=[uint8('RWW1').';be(e.uid);be(e.session_generation);e.source_system;e.source_component; ...
                hashBytes(e.execution_session_sha256);hashBytes(e.task_sha256);hashBytes(e.configuration_sha256); ...
                hashBytes(e.reference_asset_sha256);be(uint32(e.leg_index));be(w.window_generation)];
            assert(numel(manifest)==162);bytes=[manifest;body];bytes=[bytes;digest(bytes)];
            assert(numel(bytes)==28678);packets=cell(244,1);
            serviceIo();
            for k=0:243
                n=min(118,numel(bytes)-k*118);payload=zeros(128,1,'uint8');
                payload(1)=uint8(176+mod(k,16));payload(2:9)=be(w.window_generation);payload(10)=uint8(floor(k/16));
                payload(11:10+n)=bytes(k*118+1:k*118+n);
                packets{k+1}=struct('payload_type',uint16(42002),'target_system',e.source_system, ...
                    'target_component',e.source_component,'payload_length',uint8(10+n),'payload',payload);
                % Same bytes, prepared in bounded slices on the caller's
                % existing owner. No control send or lifetime starts here.
                if mod(k+1,32)==0,serviceIo();end
            end
            identity=struct('window_generation',w.window_generation,'leg_index',uint32(w.leg_index), ...
                'reference_asset_sha256',hashBytes(e.reference_asset_sha256),'message_sha256',bytes(end-31:end), ...
                'source_first_row',w.source_first_row,'row_count',w.row_count, ...
                'fragment_count',244,'message_bytes',28678,'board_receipt_proven',false,'control_authority',false);
        end
    end
end
function b=littleBytes(v),[~,~,e]=computer;if e=='B',v=swapbytes(v);end;b=reshape(typecast(v(:),'uint8'),[],1);end
function b=be(v),[~,~,e]=computer;if e=='L',v=swapbytes(v);end;b=reshape(typecast(v(:),'uint8'),[],1);end
function h=hashBytes(x)
if isa(x,'uint8'),h=x(:);else,assert((ischar(x)||isstring(x))&&strlength(string(x))==64);h=uint8(sscanf(char(x),'%2x'));end
assert(numel(h)==32&&any(h~=0),'gpenmpcNative:LocalWindowHash','Complete nonzero SHA required.');
end
function h=digest(b),m=java.security.MessageDigest.getInstance('SHA-256');m.update(typecast(b(:),'int8'));h=reshape(typecast(m.digest(),'uint8'),[],1);end
