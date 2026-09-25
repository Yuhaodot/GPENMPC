function test_gpenmpc_usb_rc_input()
% Test USB input normalization with synthetic values.
here=fileparts(mfilename('fullpath'));addpath(fullfile(fileparts(here),'host_runtime'));
c=struct('schema','GPENMPC_FS_I6S_PHYSICAL_AXIS_CALIBRATION_1', ...
    'device_name','FS-i6S emulator','vendor_id',10318,'product_id',32767,'axis_count',6, ...
    'axis_index_1based',[3 6 1 2],'positive_sign',[1 -1 1 -1], ...
    'center_raw',[0 0 0 0],'positive_span_raw',[30000 30000 30000 30000], ...
    'negative_span_raw',[28000 28000 28000 28000],'reference_deadband',.05);
s=struct('attached',true,'name',c.device_name,'vendor_id',c.vendor_id,'product_id',c.product_id, ...
    'axes_raw',zeros(1,6),'host_read_qpc_s',100);
n=0;check(s,true,[0 0 0 0]);
for k=1:4
    q=s;q.axes_raw(c.axis_index_1based(k))=c.positive_sign(k)*30000;
    expected=zeros(1,4);expected(k)=1;check(q,true,expected);
    q.axes_raw(c.axis_index_1based(k))=-c.positive_sign(k)*28000;
    expected(k)=-1;check(q,true,expected);
end
q=s;q.attached=false;check(q,false,zeros(1,4));
q=s;q.host_read_qpc_s=99;check(q,false,zeros(1,4));
q=s;q.host_read_qpc_s=101;check(q,false,zeros(1,4));
q=s;q.product_id=1;check(q,false,zeros(1,4));
q=s;q.axes_raw(3)=NaN;check(q,false,zeros(1,4));
q=s;q.axes_raw(3)=1000;check(q,true,zeros(1,4));
for heading=[0 pi/2 pi -pi/2]
    state=zeros(13,1);state(7)=cos(heading/2);state(10)=sin(heading/2);
    for stick=[1 0 0 0;0 1 0 0;-1 0 0 0;0 -1 0 0;1 1 1 1].'
        input=s;span=c.positive_span_raw;span(stick<0)=c.negative_span_raw(stick<0);
        input.axes_raw(c.axis_index_1based)=stick.'.*c.positive_sign.*span;
        r=gpenmpcNative.normalizeUsbRcInput(input,c,100.01,.1,state);
        body=[stick(2);stick(1)];body=body/max(1,norm(body));
        forward=[cos(heading);sin(heading)];right=[-sin(heading);cos(heading)];
        assert(r.valid&&norm(r.target4(2:3)-5*(forward*body(1)+right*body(2)))<1e-14);
        assert(norm(r.target4(2:3))<=5+1e-14&&r.target4(4)==1.5*stick(3));
        assert(all(abs(r.target4)<=[1;5;5;1.5])&&~r.finish_requested);n=n+1;
    end
end
input=s;input.finish_requested=true;r=gpenmpcNative.normalizeUsbRcInput(input,c,100.01,.1,state);
assert(r.valid&&r.finish_requested&&all(r.target4==0));n=n+1;
% Test timer and user-exit selection in the runner's active-loop block with a host clock.
source=fileread(fullfile(here,'run_m600_board_local_short_hil.m'));
begin=strfind(source,'    while c.duration_s==0||');ending=strfind(source,'    formalEnd=io.now();');
assert(isscalar(begin)&&isscalar(ending)&&ending>begin);loop=source(begin:ending-1);
cSaved=c;fakeTime=0;stopAt=131;service=struct('OperatorFinishRequested',false);
io=struct('now',@readClock,'sleep',@(unused)[]);firstCommit=0;cycleStart=0;remaining=0;
c=struct('duration_s',0,'poll_period_s',.001,'service_cfg',struct('operator_reference',true));
eval(loop);assert(fakeTime==131&&service.OperatorFinishRequested);n=n+1;
fakeTime=0;stopAt=Inf;service.OperatorFinishRequested=false;c.duration_s=120;
eval(loop);assert(fakeTime==120&&~service.OperatorFinishRequested);n=n+1;
fakeTime=0;stopAt=17;service.OperatorFinishRequested=false;c.duration_s=0;
eval(loop);assert(fakeTime==17&&service.OperatorFinishRequested);n=n+1;
c=cSaved;
fprintf('USB RC normalization: %d checks passed.\n',n);
    function pump()
        fakeTime=fakeTime+1;service.OperatorFinishRequested=fakeTime>=stopAt;
    end
    function t=readClock()
        t=fakeTime;
    end
    function assertOperational()
        % Test loop selection with synthetic state.
    end
    function check(input,valid,channels)
        r=gpenmpcNative.normalizeUsbRcInput(input,c,100.01,.1);
        assert(r.valid==valid&&isequal(r.channels,channels)&&~r.board_time_assigned);n=n+1;
    end
end
