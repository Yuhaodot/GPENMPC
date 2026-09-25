function result=test_board_adc_power_witness()
% Test board-ADC power observation with positive and negative host fixtures.
build=string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(build,'m600_coptersim','matlab_validation'));
status=sprintf('board_adc status\nINFO  [board_adc] running\nnsh> ');
sample=@(stamp,age,usb,valid,brick)sprintf([ ...
 '\nTOPIC: system_power instance 0 #1\n system_power\n' ...
 '    timestamp: %u (%.6f seconds ago)\n    brick_valid: %u\n' ...
 '    usb_connected: %s\n    usb_valid: %s\n'], ...
 stamp,age,brick,boolText(usb),boolText(valid));
power=[sample(1000000,0.008,true,true,0) ...
 replace(sample(1010000,0.009,true,true,0),'#1','#2')];
checks=0;failures=0;
verify(m600check.evaluateBoardAdcPowerWitness(status,power).passed);
verify(~m600check.evaluateBoardAdcPowerWitness('INFO  [board_adc] not running',power).passed);
verify(~m600check.evaluateBoardAdcPowerWitness(status,replace(power,'0.009000 seconds','0.100001 seconds')).passed);
verify(~m600check.evaluateBoardAdcPowerWitness(status,replace(power,'usb_connected: True','usb_connected: False')).passed);
verify(~m600check.evaluateBoardAdcPowerWitness(status,replace(power,'usb_valid: True','usb_valid: False')).passed);
verify(~m600check.evaluateBoardAdcPowerWitness(status,replace(power,'brick_valid: 0','brick_valid: 1')).passed);
verify(~m600check.evaluateBoardAdcPowerWitness(status,replace(power,'timestamp: 1010000','timestamp: 1000000')).passed);
verify(~m600check.evaluateBoardAdcPowerWitness(status,'never published').passed);
guard=fileread(fullfile(build,'rfly_vendor_integration','px4_runtime','BoardSafetyEvidence.hpp'));
verify(contains(guard,'usb_power_observed == GuardFact::Pass'));
outer=fileread(fullfile(build,'tools','execute_m600_board_local_short.m'));
verify(contains(outer,"'SAFE_STOP_PX4IO_START_BOARD_ADC'"));
handoff=fileread(fullfile(build,'tools','m600_local_application_handoff.m'));
verify(contains(handoff,"shellCommand('board_adc start -n',3)"));
verify(contains(handoff,"shellCommand('listener system_power -n 2',3)"));
serialLink=fileread(fullfile(build,'host_runtime','+gpenmpcNative','MavlinkSerialLink.m'));
verify(contains(serialLink,'"board_adc status"'));
verify(contains(serialLink,'"board_adc start -n"'));
verify(contains(serialLink,'"listener system_power -n 2"'));
result=struct('passed',failures==0,'checks',checks,'failures',failures, ...
 'scope','HOST_ONLY_PARSER_AND_EXACT_LIFECYCLE_BINDING','COM_opens',0, ...
 'board_actions',0,'parameter_writes',0,'arm_requests',0);
assert(result.passed,'gpenmpc:BoardAdcWitnessTest','Focused board_adc witness tests failed.');
disp(jsonencode(result));
 function verify(value),checks=checks+1;if ~value,failures=failures+1;end,end
end
function value=boolText(input),if input,value='True';else,value='False';end,end
