function receipt=runLiveHilPlantUdpService( ...
        workRoot,overlayRoot,address,port,maximumWallS,maximumMessages, ...
        readyPath,receiptPath)
% Loopback-only UDP wrapper for the single MATLAB M600 software plant.
arguments
    workRoot (1,1) string
    overlayRoot (1,1) string
    address (1,1) string = "127.0.0.1"
    port (1,1) double {mustBeInteger,mustBePositive} = 18743
    maximumWallS (1,1) double {mustBeFinite,mustBePositive} = 900
    maximumMessages (1,1) double {mustBeInteger,mustBePositive} = 250000
    readyPath (1,1) string = ""
    receiptPath (1,1) string = ""
end
assert(address=="127.0.0.1"&&isfolder(workRoot)&&isfolder(overlayRoot), ...
    'gpenmpcNative:LivePlantLoopbackOnly','Invalid service boundary.');
addpath(fullfile(workRoot,'src'));
addpath(fullfile(overlayRoot,'host_runtime'),'-begin');
rehash path;
gpenmpcNative.liveHilPlantHostService("RESET",struct(),workRoot);
server=udpport("datagram","IPV4","LocalHost",char(address), ...
    "LocalPort",port,"Timeout",2.0,"OutputDatagramSize",65507);
cleanup=onCleanup(@()closeServer(server,workRoot));
fprintf('GPENMPC_LIVE_HIL_PLANT_UDP_READY %s %d\n',address,port);
if strlength(readyPath)>0
    writeReceipt(readyPath,struct( ...
        'schema','GPENMPC_LIVE_PLANT_UDP_READY_V1', ...
        'status','READY','address',address,'port',port, ...
        'pid',matlabProcessID,'hardware_actions',0, ...
        'com_open',0,'board_access',0));
end
started=tic;messages=0;responses=0;decodeFailures=0;timeouts=0;stopped=false;
while toc(started)<maximumWallS&&messages<maximumMessages
    try
        datagram=read(server,1,"uint8");
    catch exception
        if contains(lower(string(exception.message)),"timeout")
            timeouts=timeouts+1;continue
        end
        rethrow(exception)
    end
    if isempty(datagram),timeouts=timeouts+1;continue;end
    requestId=-1;
    try
        message=jsondecode(native2unicode(uint8(datagram.Data(:).'),"UTF-8"));
        if isfield(message,'request_id'),requestId=double(message.request_id);end
        assert(isfield(message,'action'),'gpenmpcNative:LivePlantUdpAction', ...
            'Missing action.');
        payload=struct();if isfield(message,'payload'),payload=message.payload;end
        [response,stopped]=gpenmpcNative.liveHilPlantHostService( ...
            string(message.action),payload,workRoot);
    catch exception
        decodeFailures=decodeFailures+1;
        response=struct('status','FAIL_CLOSED','fail_closed',true, ...
            'failure_code',string(exception.identifier)+":"+string(exception.message), ...
            'hardware_actions',0,'com_open',0,'board_access',0);
    end
    response.request_id=requestId;
    encoded=unicode2native(jsonencode(response),'UTF-8');
    assert(numel(encoded)<=65507,'gpenmpcNative:LivePlantUdpResponseSize', ...
        'Response exceeds one datagram.');
    write(server,uint8(encoded),'uint8', ...
        datagram.SenderAddress,datagram.SenderPort);
    messages=messages+1;responses=responses+1;
    if stopped,break;end
end
receipt=struct('schema','GPENMPC_LIVE_PLANT_UDP_RECEIPT_V1', ...
    'address',address,'port',port,'messages',messages,'responses',responses, ...
    'decode_failures',decodeFailures,'read_timeouts',timeouts, ...
    'stopped_by_client',stopped,'wall_s',toc(started), ...
    'hardware_actions',0,'com_open',0,'board_access',0);
if strlength(receiptPath)>0,writeReceipt(receiptPath,receipt);end
end

function writeReceipt(path,value)
path=string(path);parent=fileparts(path);
if strlength(parent)>0&&~isfolder(parent),mkdir(parent);end
temporary=path+".tmp";
fid=fopen(temporary,'w','n','UTF-8');
assert(fid>=0,'gpenmpcNative:LivePlantReceiptOpen', ...
    'Unable to open service receipt.');
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(value,PrettyPrint=true));
clear cleanup
movefile(temporary,path,'f');
end

function closeServer(server,workRoot)
try
    gpenmpcNative.liveHilPlantHostService("RESET",struct(),workRoot);
catch
end
try
    flush(server);
catch
end
try
    delete(server);
catch
end
end
