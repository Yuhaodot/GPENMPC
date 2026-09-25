function command=normalizeUsbRcInput(sample,calibration,hostNowQpcS,maxReadAgeS,canonicalState13)
% Normalize operator input in channel order:
% right-right, right-forward, left-forward, left-right.
assert(strcmp(calibration.schema,'GPENMPC_FS_I6S_PHYSICAL_AXIS_CALIBRATION_1'));
assert(isscalar(hostNowQpcS)&&isfinite(hostNowQpcS)&&isfinite(maxReadAgeS)&&maxReadAgeS>0);
command=struct('valid',false,'channels',zeros(1,4),'reason','invalid_sample', ...
    'host_read_qpc_s',NaN,'board_time_assigned',false);
if ~sample.attached,command.reason='USB_disconnected';return;end
if sample.vendor_id~=calibration.vendor_id||sample.product_id~=calibration.product_id ...
        ||~strcmp(sample.name,calibration.device_name)||numel(sample.axes_raw)~=calibration.axis_count
    command.reason='device_mismatch';return
end
age=hostNowQpcS-sample.host_read_qpc_s;
if ~isfinite(age)||age<0||age>maxReadAgeS,command.reason='host_read_not_current';return;end
index=calibration.axis_index_1based;
assert(numel(index)==4&&numel(unique(index))==4&&all(index>=1&index<=numel(sample.axes_raw)) ...
    &&all(index==fix(index))&&all(ismember(calibration.positive_sign,[-1 1])) ...
    &&all(calibration.positive_span_raw>0)&&all(calibration.negative_span_raw>0));
d=(double(sample.axes_raw(index))-calibration.center_raw).*calibration.positive_sign;
if any(~isfinite(d)),command.reason='nonfinite_input';return;end
scale=calibration.positive_span_raw;scale(d<0)=calibration.negative_span_raw(d<0);
u=d./scale;
if any(abs(u)>1.15),command.reason='outside_calibrated_range';return;end
u=min(max(u,-1),1);deadband=calibration.reference_deadband;
assert(isscalar(deadband)&&isfinite(deadband)&&deadband>=0&&deadband<1);
u=sign(u).*max(0,(abs(u)-deadband)/(1-deadband));
command.valid=true;command.channels=u;command.reason='valid_operator_reference';
command.host_read_qpc_s=sample.host_read_qpc_s;
command.finish_requested=isfield(sample,'finish_requested')&&isequal(sample.finish_requested,true);
if nargin>=5
    % With NED mapping C=diag(1,1,-1) and q=[w,-x,-y,z], heading is unchanged.
    % Rotate operator input with right=[-sin(psi),cos(psi)].
    state=double(canonicalState13(:));assert(numel(state)==13&&all(isfinite(state)));
    q=state(7:10);assert(abs(sum(q.^2)-1)<1e-6,'gpenmpcRc:HeadingQuaternion');
    heading=atan2(2*(q(1)*q(4)+q(2)*q(3)),1-2*(q(3)^2+q(4)^2));
    horizontal=[u(2);u(1)];horizontal=horizontal/max(1,norm(horizontal));
    world=[cos(heading) -sin(heading);sin(heading) cos(heading)]*horizontal;
    % Manual-reference limits: horizontal vector <=5 m/s,
    % vertical <=1.5 m/s and yaw <=1 rad/s.
    command.target4=[u(4);5*world;1.5*u(3)];
    command.heading_rad=heading;
    command.reference_source='USB_OPERATOR_BODY_HEADING_VELOCITY_AND_YAW_RATE';
end
end
