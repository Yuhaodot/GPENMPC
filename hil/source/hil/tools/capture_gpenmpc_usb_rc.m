function result=capture_gpenmpc_usb_rc(duration,outputRoot)
% Capture USB input samples.
arguments
    duration (1,1) double {mustBePositive}=25
    outputRoot (1,1) string=""
end
here=fileparts(mfilename('fullpath'));addpath(fullfile(here,'usb_rc_runtime'));
guard=onCleanup(@()gpenmpc_usb_joystick_mex('close')); %#ok<NASGU>
info=gpenmpc_usb_joystick_mex('open',gpenmpc_install_path('rfly','QGroundControl\SDL2.dll'));
fprintf('USB_INPUT_CONNECTED %s axes=%d buttons=%d\n',info.name,numel(info.axes_raw),numel(info.buttons));
capacity=ceil(duration*100)+100;raw=nan(capacity,numel(info.axes_raw));
buttons=false(capacity,numel(info.buttons));times=nan(capacity,1);cost=times;
n=0;clock=tic;nextPrint=0;
while toc(clock)<duration
    call=tic;s=gpenmpc_usb_joystick_mex('read');elapsed=toc(call);
    assert(s.attached,'gpenmpcRc:Disconnected','USB transmitter disconnected.');
    n=n+1;assert(n<=capacity);raw(n,:)=s.axes_raw;buttons(n,:)=s.buttons;times(n)=s.host_read_qpc_s;cost(n)=elapsed;
    if toc(clock)>=nextPrint
        fprintf('USB_INPUT t=%.2f axes=%s buttons=%s\n',toc(clock),mat2str(s.axes_raw),mat2str(find(s.buttons)));
        nextPrint=nextPrint+2;
    end
    pause(.02);
end
result=struct('scope','USB_HID_CAPTURE','device',info,'samples',n, ...
    'raw_axes',raw(1:n,:),'buttons',buttons(1:n,:),'host_read_qpc_s',times(1:n), ...
    'read_call_s',cost(1:n),'minimum',min(raw(1:n,:),[],1), ...
    'maximum',max(raw(1:n,:),[],1),'COM_open',0,'arm_commands',0,'control_submissions',0);
if strlength(outputRoot)==0,outputRoot=fullfile(here,'usb_rc_runtime');end
assert(isfolder(outputRoot),'gpenmpcRc:OutputDirectory','Use an existing output directory.');
out=fullfile(outputRoot,['USB_INPUT_' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')) '.mat']);
save(out,'result');
fprintf('USB_INPUT_RESULT samples=%d min=%s max=%s max_read_ms=%.4f path=%s\n', ...
    n,mat2str(result.minimum),mat2str(result.maximum),max(cost(1:n))*1000,out);
end
