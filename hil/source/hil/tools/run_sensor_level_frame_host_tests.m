function result=run_sensor_level_frame_host_tests(outputPath)
% Test sensor-frame math and source consistency.
assert(~isfile(outputPath),'m600check:FrameTestExists');
root=fileparts(fileparts(mfilename('fullpath')));folder=fullfile(root,'m600_coptersim','matlab_validation');addpath(folder);
names={};passed=[];
x=6.445373058319092*pi/180;y=-6.442468166351318*pi/180;
Rx=[1,0,0;0,cos(x),-sin(x);0,sin(x),cos(x)];
Ry=[cos(y),0,sin(y);0,1,0;-sin(y),0,cos(y)];oracle=Ry*Rx;
vectors={[0;0;-9.80665],[1;2;3],[-4.6;.71;-1.2],[.2194;.023;.417],[0;0;0]};
unchanged=[1,11:30];
for k=1:numel(vectors)
    body=(1:30).';body(2:4)=vectors{k};body(5:7)=vectors{mod(k,5)+1};body(8:10)=vectors{mod(k+1,5)+1};
    [wire,L]=m600check.encodeHilSensorLevelFrame(body);
    check(sprintf('case%d_all_three_sensor_vectors_inverse',k),max(abs(wire(2:10)-reshape(oracle.'*reshape(body(2:10),3,3),9,1)))<1e-13);
    check(sprintf('case%d_PX4_recalibration_recovers_body',k),max(abs(reshape(oracle*reshape(wire(2:10),3,3),9,1)-body(2:10)))<1e-12);
    check(sprintf('case%d_21_channels_bitwise_unchanged',k),isequal(typecast(wire(unchanged),'uint64'),typecast(body(unchanged),'uint64')));
end
check('rotation_from_independent_axis_product',max(abs(L(:)-oracle(:)))<1e-15);
check('L_orthonormal_positive_determinant',norm(L*L.'-eye(3),'fro')<1e-14&&abs(det(L)-1)<1e-14);
body=zeros(30,1);body(2:4)=[0;0;-9.80665];wire=m600check.encodeHilSensorLevelFrame(body);
check('neutral_raw_with_existing_level_would_tilt',norm(oracle*body(2:4)-body(2:4))>1);
check('single_inverse_removes_that_tilt',norm(oracle*wire(2:4)-body(2:4))<1e-12);
check('double_inverse_is_not_accepted_as_identity',norm(oracle*oracle.'*wire(2:4)-body(2:4))>1);
bits=typecast(body,'uint64');bits(1)=bitor(bitshift(uint64(1),63),uint64(0));bits(11)=bitor(uint64(9221120237041090560),uint64(1));body=typecast(bits,'double');
wire=m600check.encodeHilSensorLevelFrame(body);
check('signed_zero_and_NaN_payload_other_channels_preserved',isequal(typecast(wire(unchanged),'uint64'),typecast(body(unchanged),'uint64')));
for k=1:3
    rejected=false;bad={zeros(30,1,'single'),zeros(1,30),zeros(29,1)};
    try,m600check.encodeHilSensorLevelFrame(bad{k});catch,rejected=true;end
    check(sprintf('invalid_wire_shape_type_%d_rejected',k),rejected);
end
result=struct('status','HOST_ONLY_SENSOR_LEVEL_FRAME_UNIT_TESTS','passed',all(passed), ...
    'checks_total',numel(passed),'checks_passed',sum(passed),'checks',struct('name',names,'passed',num2cell(passed)), ...
    'helper_sha256',m600check.fileSha256(fullfile(folder,'+m600check','encodeHilSensorLevelFrame.m')), ...
    'required_board_level_deg',[6.445373058319092,-6.442468166351318,0], ...
    'independent_axis_product_L',oracle,'COM_open',0,'board_actions',0,'simulator_actions',0,'DLL_build',0,'HIL',false);
parent=fileparts(outputPath);if ~isfolder(parent),mkdir(parent);end
fid=fopen(outputPath,'w','n','UTF-8');assert(fid>=0);f=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(result,PrettyPrint=true));clear f
disp(jsonencode(struct('passed',result.passed,'checks_total',result.checks_total,'checks_passed',result.checks_passed)));
assert(result.passed,'m600check:FrameHostTestsFailed');
    function check(n,v),names{end+1}=n;passed(end+1)=logical(v);end
end
