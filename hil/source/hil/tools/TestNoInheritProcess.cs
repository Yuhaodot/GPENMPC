using System;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Threading;
using GPENMPC.HostDiagnostics;

// One isolated HOST-only fixture. Child only checks inputs and waits on named events.
internal static class TestNoInheritProcess
{
    [DllImport("kernel32.dll", SetLastError = true)]
    private static extern bool SetHandleInformation(IntPtr handle, uint mask, uint flags);
    [DllImport("kernel32.dll", SetLastError = true)]
    private static extern bool GetHandleInformation(IntPtr handle, out uint flags);
    private static void Require(bool ok, string reason) { if (!ok) throw new Exception(reason); }
    public static int Main(string[] args)
    {
        if (args.Length > 0 && args[0] == "--child")
        {
            if (args.Length != 4 || args[3] != "argument with spaces") return 10;
            if (Environment.GetEnvironmentVariable("GPENMPC_NO_INHERIT_FIXTURE") != "value with spaces \u4e2d\u6587") return 11;
            if (Environment.GetEnvironmentVariable("GPENMPC_NO_INHERIT_CWD") != Environment.CurrentDirectory) return 12;
            using (var ready = EventWaitHandle.OpenExisting(args[1]))
            using (var release = EventWaitHandle.OpenExisting(args[2]))
            { ready.Set(); return release.WaitOne(15000) ? 0 : 13; }
        }
        string prefix = "Local\\GPENMPC_NoInherit_" + Guid.NewGuid().ToString("N");
        // The .NET Framework environment dictionary rejects duplicate Path/PATH keys.
        // Remove PATH only in this disposable test process.
        Environment.SetEnvironmentVariable("PATH", null);
        Process child = null;
        Socket original = null, rebound = null;
        using (var ready = new EventWaitHandle(false, EventResetMode.ManualReset, prefix + "_ready"))
        using (var release = new EventWaitHandle(false, EventResetMode.ManualReset, prefix + "_release"))
        {
            try
            {
                original = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
                original.ExclusiveAddressUse = true;
                original.Bind(new IPEndPoint(IPAddress.Loopback, 0));
                int port = ((IPEndPoint)original.LocalEndPoint).Port;
                Require(SetHandleInformation(original.Handle, 1, 1), "Cannot deliberately make fixture UDP socket inheritable.");
                uint flags;
                Require(GetHandleInformation(original.Handle, out flags) && (flags & 1) == 1, "Socket inherit flag not established.");
                var info = new ProcessStartInfo();
                info.FileName = System.Reflection.Assembly.GetExecutingAssembly().Location;
                info.Arguments = "--child \"" + prefix + "_ready\" \"" + prefix + "_release\" \"argument with spaces\"";
                info.WorkingDirectory = Environment.CurrentDirectory;
                info.UseShellExecute = false;
                info.CreateNoWindow = true;
                info.EnvironmentVariables["GPENMPC_NO_INHERIT_FIXTURE"] = "value with spaces \u4e2d\u6587";
                info.EnvironmentVariables["GPENMPC_NO_INHERIT_CWD"] = Environment.CurrentDirectory;
                child = NoInheritProcess.Start(info);
                Require(ready.WaitOne(5000), "Harmless child did not confirm arguments/environment/cwd.");
                Require(!child.HasExited, "Child must be alive during original socket release.");
                original.Dispose(); original = null;
                rebound = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
                rebound.ExclusiveAddressUse = true;
                rebound.Bind(new IPEndPoint(IPAddress.Loopback, port));
                Require(!child.HasExited, "Child must remain alive during exclusive rebind.");
                release.Set();
                Require(child.WaitForExit(5000) && child.ExitCode == 0, "Harmless child did not finish cleanly.");
                Console.WriteLine("PASS_HOST_ONLY_NO_HANDLE_INHERIT: inheritable_udp=true; parent_socket_closed=true; exclusive_rebind_while_child_alive=true; args_env_cwd=true; child_exit=0; port=" + port + "; child_pid=" + child.Id + "; COM=0; CopterSim=0; board=0; HIL=0");
                return 0;
            }
            catch (Exception ex) { Console.Error.WriteLine(ex.ToString()); return 1; }
            finally
            {
                release.Set();
                if (child != null) { if (!child.HasExited && !child.WaitForExit(2000)) { child.Kill(); child.WaitForExit(2000); } child.Dispose(); }
                if (original != null) original.Dispose();
                if (rebound != null) rebound.Dispose();
            }
        }
    }
}
