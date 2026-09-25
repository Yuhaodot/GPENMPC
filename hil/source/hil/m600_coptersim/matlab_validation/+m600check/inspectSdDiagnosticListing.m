function r=inspectSdDiagnosticListing(text)
% Read-only NSH directory response. Never rename/remove a board diagnostic.
text=char(text);
echo=strfind(text,'ls /fs/microsd');complete=false;body='';
if ~isempty(echo)
    afterEcho=text(echo(end)+numel('ls /fs/microsd'):end);
    terminal=strfind(afterEcho,'nsh>');
    if ~isempty(terminal),complete=true;body=afterEcho(1:terminal(1)-1);end
end
header=~isempty(regexp(body,'(?m)^\s*/fs/microsd/?\s*:\s*$','once'));
errorReported=~isempty(regexpi(body,'(?:failed|no such file|permission denied|not found|I/O error)','once'));
faults=regexp(body,'(?:^|[\s/])(fault_[A-Za-z0-9_.-]+\.log)(?=\s|$)','tokens');
names=cellfun(@(v)v{1},faults,'UniformOutput',false);names=unique(names,'stable');
r=struct('response',text,'shell_completed',complete,'directory_header_observed',header, ...
    'error_reported',errorReported,'active_fault_log_names',{names}, ...
    'passed',complete&&header&&~errorReported&&isempty(names), ...
    'file_service_actions',0,'scope','TOP_LEVEL_DIRECTORY_ONLY__ARCHIVED_NAMES_NOT_CURRENT_FAULTS');
end
