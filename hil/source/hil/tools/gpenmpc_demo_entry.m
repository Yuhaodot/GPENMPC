function gpenmpc_demo_entry()
% One persistent viewer; separate hardware owner is launched only by Start.
focusEvent=System.Threading.EventWaitHandle.OpenExisting('Local\GPENMPCHILDemoFocus');
readyEvent=System.Threading.EventWaitHandle.OpenExisting('Local\GPENMPCHILDemoReady');
cleanup=onCleanup(@()releaseEvents(focusEvent,readyEvent)); %#ok<NASGU>
gpenmpc_demo_console('LIVE','EN');drawnow;readyEvent.Set();
while true
    f=findall(groot,'Type','figure','Tag','GPENMPCDemoConsole');
    if isempty(f),break;end
    if focusEvent.WaitOne(0)
        f(1).WindowState='normal';figure(f(1));
        % Reopen the dashboard in response to the explicit action.
        if isappdata(f(1),'GPENMPCShow3DView')
            showView=getappdata(f(1),'GPENMPCShow3DView');showView();
        end
    end
    pause(.1);
end
end
function releaseEvents(focusEvent,readyEvent)
focusEvent.Dispose();readyEvent.Dispose();
end
