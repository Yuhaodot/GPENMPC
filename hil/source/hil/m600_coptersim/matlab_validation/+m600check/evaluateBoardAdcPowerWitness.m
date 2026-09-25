function evidence=evaluateBoardAdcPowerWitness(statusText,powerText)
% Pure parser for the stock PX4 board_adc/system_power shell evidence.
% It grants no authority and performs no board, serial, parameter or process IO.
statusText=char(string(statusText));
powerText=char(string(powerText));
evidence=struct('passed',false,'status_running',false,'sample_count',0, ...
 'timestamps_us',[],'ages_s',[],'usb_connected',[],'usb_valid',[], ...
 'brick_valid',[],'fresh_max_age_s',0.1,'failures',{{}}, ...
 'scope','PURE_HOST_PARSE_OF_BOARD_ADC_STATUS_AND_SYSTEM_POWER');
evidence.status_running=~isempty(regexp(statusText,'(?i)\[board_adc\]\s+running(?:\s|$)','once')) ...
 &&isempty(regexp(statusText,'(?i)\[board_adc\]\s+not running(?:\s|$)','once'));
evidence.timestamps_us=parseNumbers(powerText,'(?mi)^\s*timestamp:\s*([0-9]+)');
evidence.ages_s=parseNumbers(powerText,'(?mi)^\s*timestamp:\s*[0-9]+\s*\(\s*([0-9eE+\-.]+)\s+seconds?\s+ago\s*\)');
evidence.usb_connected=parseBooleans(powerText,'usb_connected');
evidence.usb_valid=parseBooleans(powerText,'usb_valid');
evidence.brick_valid=parseNumbers(powerText,'(?mi)^\s*brick_valid:\s*([0-9]+)');
evidence.sample_count=numel(evidence.timestamps_us);
failures={};
if ~evidence.status_running,failures{end+1}='BOARD_ADC_NOT_RUNNING';end %#ok<AGROW>
if contains(lower(powerText),'never published')||contains(lower(powerText),'without a message')
 failures{end+1}='SYSTEM_POWER_NOT_PUBLISHED'; %#ok<AGROW>
end
if evidence.sample_count~=2||any(evidence.timestamps_us<=0)||any(diff(evidence.timestamps_us)<=0)
 failures{end+1}='TWO_DISTINCT_NONZERO_TIMESTAMPS_REQUIRED'; %#ok<AGROW>
end
if numel(evidence.ages_s)~=2||any(~isfinite(evidence.ages_s))||any(evidence.ages_s<0)|| ...
   any(evidence.ages_s>evidence.fresh_max_age_s)
 failures{end+1}='SYSTEM_POWER_NOT_FRESH'; %#ok<AGROW>
end
if numel(evidence.usb_connected)~=2||~all(evidence.usb_connected)
 failures{end+1}='USB_CONNECTED_NOT_TRUE'; %#ok<AGROW>
end
if numel(evidence.usb_valid)~=2||~all(evidence.usb_valid)
 failures{end+1}='USB_VALID_NOT_TRUE'; %#ok<AGROW>
end
if numel(evidence.brick_valid)~=2||any(evidence.brick_valid~=0)
 failures{end+1}='BRICK_POWER_PRESENT_OR_UNKNOWN'; %#ok<AGROW>
end
evidence.failures=failures;evidence.passed=isempty(failures);
end

function values=parseNumbers(text,pattern)
tokens=regexp(text,pattern,'tokens');
values=zeros(1,numel(tokens));
for k=1:numel(tokens),values(k)=str2double(tokens{k}{1});end
end

function values=parseBooleans(text,name)
tokens=regexp(text,['(?mi)^\s*' name ':\s*(True|False|1|0)\s*$'],'tokens');
values=false(1,numel(tokens));
for k=1:numel(tokens),values(k)=any(strcmpi(tokens{k}{1},{'true','1'}));end
end
