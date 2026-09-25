function [bytes,fields]=encodePlantEnvironment(value,copterId,initialPayloadKg)
% Pure host encoder for the official CopterSim inDoubCtrls[28] input.
% NO socket, controller input, plant instance, or payload change is made here.
% Future DLL consumer MUST separately prove generation/freshness and permit
% unloading only after actual ground/disarm/dwell. This codec alone does not.
% Official SDK DllSimCtrlAPI.py sendInDoubCtrls: int32 checksum1234567897,
% int32 CopterID, 28 float64 little-endian, UDP30100+2*(ID-1).
% Reference here drives the SOURCE plant's structural-residual environment;
% it is not plant truth and cannot replace EKF/atomic state at the controller.
assert(isscalar(copterId)&&isfinite(copterId)&&copterId==fix(copterId)&&copterId>=1&&copterId<=255);
assert(isscalar(initialPayloadKg)&&isfinite(initialPayloadKg)&&initialPayloadKg>=0);
required={'schema','generation','source_io_time_s','task_reference_time_s', ...
    'payload_kg','wind_ned_xy_mps','mission_phase','reference_jet_ned', ...
    'payload_generation','task_clock_paused'};
assert(isstruct(value)&&isscalar(value)&&all(isfield(value,required)), ...
    'gpenmpcTaskIo:EnvironmentSchema','Incomplete environment frame.');
assert(strcmp(value.schema,'M600_PLANT_ENVIRONMENT_V1'),'gpenmpcTaskIo:EnvironmentSchema','Wrong schema.');
integer(value.generation,1,2^32-1);integer(value.payload_generation,0,2^32-1);
integer(value.mission_phase,0,6);
assert(islogical(value.task_clock_paused)&&isscalar(value.task_clock_paused));
scalar(value.source_io_time_s);scalar(value.task_reference_time_s);scalar(value.payload_kg);
assert(value.source_io_time_s>=0&&value.task_reference_time_s>=0&& ...
    value.payload_kg>=0&&value.payload_kg<=initialPayloadKg,'gpenmpcTaskIo:EnvironmentRange','Invalid time/payload.');
assert(isnumeric(value.wind_ned_xy_mps)&&numel(value.wind_ned_xy_mps)==2&& ...
    isreal(value.wind_ned_xy_mps)&&all(isfinite(value.wind_ned_xy_mps(:))));
assert(isnumeric(value.reference_jet_ned)&&numel(value.reference_jet_ned)==12&& ...
    isreal(value.reference_jet_ned)&&all(isfinite(value.reference_jet_ned(:))));
fields=zeros(1,28);
fields(1:8)=[1,double(value.generation),double(value.source_io_time_s), ...
    double(value.task_reference_time_s),double(value.payload_kg), ...
    reshape(double(value.wind_ned_xy_mps),1,2),double(value.mission_phase)];
fields(9:20)=reshape(double(value.reference_jet_ned),1,12);
fields(21:22)=[double(value.payload_generation),double(value.task_clock_paused)];
bytes=[packLE(int32([1234567897,copterId])),packLE(fields)];
assert(isa(bytes,'uint8')&&numel(bytes)==232);
end
function scalar(v),assert(isnumeric(v)&&isscalar(v)&&isreal(v)&&isfinite(v));end
function integer(v,a,b),scalar(v);assert(v==fix(v)&&v>=a&&v<=b);end
function b=packLE(v)
[~,~,e]=computer;if e=='B',v=swapbytes(v);end
b=reshape(typecast(v,'uint8'),1,[]);
end
