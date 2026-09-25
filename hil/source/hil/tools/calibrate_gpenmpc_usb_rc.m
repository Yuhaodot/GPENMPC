function calibrate_gpenmpc_usb_rc()
% Calibrate transmitter axes from interactive USB HID input.
here=fileparts(mfilename('fullpath'));out=fullfile(here,'usb_rc_runtime');addpath(out);
cleanup=onCleanup(@finish); %#ok<NASGU>
info=gpenmpc_usb_joystick_mex('open',gpenmpc_install_path('rfly','QGroundControl\SDL2.dll'));
fig=figure('Name','USB transmitter axis calibration', ...
    'NumberTitle','off','MenuBar','none','ToolBar','none','Position',[120 160 860 540]);
instruction=uicontrol(fig,'Style','text','Units','normalized','Position',[.04 .83 .92 .13], ...
    'FontSize',16,'HorizontalAlignment','left');
status=uicontrol(fig,'Style','text','Units','normalized','Position',[.04 .73 .92 .09], ...
    'FontSize',11,'HorizontalAlignment','left','String','Move one stick axis at a time to calibrate USB input.');
ax=axes(fig,'Position',[.09 .28 .85 .40]);plotBars=bar(ax,zeros(1,numel(info.axes_raw)));
ylim(ax,[-32768 32767]);xticks(ax,1:numel(info.axes_raw));xlabel(ax,'USB axis index');ylabel(ax,'Raw value');grid(ax,'on');
next=uicontrol(fig,'Style','pushbutton','Units','normalized','Position',[.54 .07 .40 .11], ...
    'FontSize',13,'String','Record position','Callback',@accept);
uicontrol(fig,'Style','pushbutton','Units','normalized','Position',[.05 .07 .40 .11], ...
    'FontSize',13,'String','Restart','Callback',@reset);
step=0;latest=[];baseline=[];mapping=zeros(1,4);polarity=zeros(1,4);fullscale=zeros(1,4);
lastError='';observations=zeros(0,numel(info.axes_raw));sampleCount=0;costMax=0;
positivePrompts={'Move the right stick fully right; center other axes, hold, then select Record position.', ...
    'Center the right stick, move it fully forward, hold, then select Record position.', ...
    'Center the right stick, move the left stick fully forward, hold, then select Record position.', ...
    'Center the left vertical axis, move the left stick fully right, hold, then select Record position.'};
negativePrompts={'Move the right stick fully left; center other axes, hold, then select Record position.', ...
    'Center the right stick, move it fully back, hold, then select Record position.', ...
    'Center the right stick, move the left stick fully back, hold, then select Record position.', ...
    'Center the left vertical axis, move the left stick fully left, hold, then select Record position.'};
negativeScale=zeros(1,4);updateText();
while isgraphics(fig)
    call=tic;
    try
        latest=gpenmpc_usb_joystick_mex('read');costMax=max(costMax,toc(call));sampleCount=sampleCount+1;
        if ~latest.attached
            set(status,'String','USB disconnected. Close the window and reconnect the transmitter.');set(next,'Enable','off');
        else
            set(plotBars,'YData',latest.axes_raw);
            set(status,'String',sprintf('%s | %d reads | Maximum read %.3f ms | Buttons %s\n%s', ...
                latest.name,sampleCount,1000*costMax,mat2str(find(latest.buttons)),lastError));
        end
    catch err
        set(status,'String',['Read failed: ' err.message]);set(next,'Enable','off');
    end
    drawnow limitrate;pause(.025);
end
    function updateText()
        if step==0
            text='Step 1: Center both sticks and select Record position.';
        elseif step<=4
            text=sprintf('Step %d: %s',step+1,positivePrompts{step});
        elseif step<=8
            text=sprintf('Step %d: %s',step+1,negativePrompts{step-4});
        else
            text='Final step: Center both sticks and select Record position to save.';
        end
        set(instruction,'String',text);
    end
    function reset(~,~)
        step=0;baseline=[];mapping(:)=0;polarity(:)=0;fullscale(:)=0;negativeScale(:)=0;
        observations=zeros(0,numel(info.axes_raw));lastError='';set(next,'Enable','on');updateText();
    end
    function accept(~,~)
        if isempty(latest)||~latest.attached,return;end
        raw=latest.axes_raw;
        if step==0
            baseline=raw;
        elseif step<=4
            delta=raw-baseline;[largest,index]=max(abs(delta));others=abs(delta);others(index)=0;
            % Require deliberate single-axis motion when calibrating input mapping.
            if largest<10000||max(others)>.25*largest||any(mapping==index)
                lastError='Move only the indicated axis through its full range.';return
            end
            mapping(step)=index;polarity(step)=sign(delta(index));fullscale(step)=largest;
        elseif step<=8
            channel=step-4;delta=raw-baseline;value=delta(mapping(channel))*polarity(channel);
            other=abs(delta);other(mapping(channel))=0;
            if value>-10000||max(other)>.25*abs(value)
                lastError='Direction or center mismatch. Move the indicated stick to the opposite endpoint.';return
            end
            negativeScale(channel)=-value;
        else
            if any(abs(raw(mapping)-baseline(mapping))> .10*min(fullscale,negativeScale))
                lastError='Center all four axes before saving.';return
            end
            calibration=struct('schema','GPENMPC_FS_I6S_PHYSICAL_AXIS_CALIBRATION_1', ...
                'device_name',info.name,'vendor_id',info.vendor_id,'product_id',info.product_id, ...
                'axis_count',numel(info.axes_raw),'channel_order',{{'right_right','right_forward','left_forward','left_right'}}, ...
                'axis_index_1based',mapping,'positive_sign',polarity,'center_raw',baseline(mapping), ...
                'positive_span_raw',fullscale,'negative_span_raw',negativeScale, ...
                'raw_positions',[observations;raw],'completed_at',char(datetime('now')), ...
                'reference_deadband',.05,'armed',false,'HIL_verified',false, ...
                'scope','OPERATOR_INPUT_CALIBRATION_ONLY_NO_CONTROLLER_OR_MODEL_CHANGE');
            path=fullfile(out,['FS_I6S_CALIBRATION_' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')) '.mat']);
            save(path,'calibration');lastError=['Saved: ' path];set(next,'Enable','off');
            set(instruction,'String','Four-axis calibration saved.');step=10;return
        end
        observations(end+1,:)=raw;step=step+1;lastError='';updateText();
    end
    function finish()
        gpenmpc_usb_joystick_mex('close');
    end
end
