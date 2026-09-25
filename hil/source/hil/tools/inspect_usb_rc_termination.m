function inspect_usb_rc_termination(runRoot)
% Read termination variables without loading full flight records.
a=load(fullfile(runRoot,'SHORT_HIL','RAW_BOARD_LOCAL_SHORT_HIL.mat'),'sessionRaw','events','commandRaw');
disp('RECORDED EVENTS');
for k=1:numel(a.events),disp(jsonencode(a.events{k}));end
disp('RECORDED COMMANDS');
for k=1:numel(a.commandRaw)
 c=a.commandRaw{k};disp(jsonencode(c));
end
disp('POST-RELEASE BOARD CONSOLE');
frames=a.sessionRaw.post_release_evidence_frames;
parts=cell(size(frames));
for k=1:numel(frames)
 if iscell(frames),f=frames{k};else,f=frames(k);end
 b=uint8(f.raw_frame(:));
 if b(1)==253,offset=10;else,offset=6;end
 p=b(offset+(1:double(b(2))));
 if numel(p)>=9,n=double(p(9));n=min(n,numel(p)-9);parts{k}=char(p(10:9+n).');end
end
fprintf('%s\n',[parts{:}]);
end
