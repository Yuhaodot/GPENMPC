function mirror=makeDisplayTruthMirror(port)
% Display-only copy of official truth packets. Never a control input.
% Nonblocking loopback UDP; at most 5 Hz, no ACK/retry, no renderer in this owner.
% Display failure disables this optional output independently of logging.
socket=[];destination=[];last=-Inf;sent=0;disabled=false;lastError='';maxSendS=0;
mirror=struct('send',@send,'due',@(nowS)~disabled&&nowS-last>=.2,'close',@closeMirror,'status',@status);
if isempty(port)||port==0,disabled=true;return;end
assert(isscalar(port)&&port==30251,'m600check:DisplayPort','Only the display-only loopback port is allowed.');
try
 socket=System.Net.Sockets.Socket(System.Net.Sockets.AddressFamily.InterNetwork, ...
   System.Net.Sockets.SocketType.Dgram,System.Net.Sockets.ProtocolType.Udp);
 socket.Blocking=false;
 destination=System.Net.IPEndPoint(System.Net.IPAddress.Loopback,int32(port));
 % Warm marshaling/send before the plant starts. Empty datagram is NOT a state.
 socket.SendTo(uint8([]),destination);
catch ex
 disabled=true;lastError=ex.message;closeMirror();
end
    function send(bytes,nowS,extra)
        if nargin<3,extra={};end
        if disabled||nowS-last<.2,return;end
        last=nowS;t=tic;
        try
            % Environment and diagnostic packets for the display.
            for k=1:numel(extra),socket.SendTo(uint8(extra{k}(:)),destination);end
            socket.SendTo(uint8(bytes(:)),destination);sent=sent+1;
        catch ex
            disabled=true;lastError=ex.message;closeMirror();
        end
        maxSendS=max(maxSendS,toc(t));
    end
    function closeMirror()
        if ~isempty(socket),try,socket.Close();catch,end;socket=[];end
        disabled=true;
    end
    function value=status()
        value=struct('display_only',true,'port',port,'sent',sent, ...
          'disabled',disabled,'max_send_s',maxSendS,'last_error',lastError);
    end
end
