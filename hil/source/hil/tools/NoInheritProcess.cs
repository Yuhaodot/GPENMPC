using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;

namespace GPENMPC.HostDiagnostics
{
    // Launch a child without inheriting parent handles.
    public static class NoInheritProcess
    {
        [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
        private struct StartupInfo
        {
            public int cb;
            public string reserved, desktop, title;
            public int x, y, xSize, ySize, xCountChars, yCountChars, fillAttribute, flags;
            public short showWindow, reservedBytes;
            public IntPtr reservedPointer, stdInput, stdOutput, stdError;
        }
        [StructLayout(LayoutKind.Sequential)]
        private struct ProcessInformation
        {
            public IntPtr process, thread;
            public int processId, threadId;
        }
        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        private static extern bool CreateProcessW(string application, StringBuilder commandLine,
            IntPtr processAttributes, IntPtr threadAttributes,
            [MarshalAs(UnmanagedType.Bool)] bool inheritHandles, uint creationFlags,
            IntPtr environment, string currentDirectory, ref StartupInfo startup,
            out ProcessInformation processInformation);
        [DllImport("kernel32.dll", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        private static extern bool CloseHandle(IntPtr handle);
        [DllImport("kernel32.dll", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        private static extern bool TerminateProcess(IntPtr process, uint exitCode);
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern uint WaitForSingleObject(IntPtr handle, uint milliseconds);

        // Expose Process.Id, HasExited, Kill and WaitForExit to the caller.
        // Forward arguments and the explicit ProcessStartInfo environment.
        public static Process Start(ProcessStartInfo info)
        {
            if (info == null) throw new ArgumentNullException("info");
            if (info.UseShellExecute || info.RedirectStandardInput || info.RedirectStandardOutput || info.RedirectStandardError)
                throw new ArgumentException("Direct launch without redirected standard handles is required.");
            if (!Path.IsPathRooted(info.FileName) || !File.Exists(info.FileName) || info.FileName.IndexOf('"') >= 0)
                throw new ArgumentException("An exact existing absolute executable path is required.");
            if (!Path.IsPathRooted(info.WorkingDirectory) || !Directory.Exists(info.WorkingDirectory))
                throw new ArgumentException("An exact existing absolute working directory is required.");
            if (!String.IsNullOrEmpty(info.UserName))
                throw new ArgumentException("Alternate-user launch is not supported.");
            var keys = new List<string>();
            foreach (string key in info.EnvironmentVariables.Keys) keys.Add(key);
            keys.Sort(StringComparer.OrdinalIgnoreCase);
            var env = new StringBuilder();
            foreach (string key in keys)
            {
                string value = info.EnvironmentVariables[key] ?? "";
                if (key.IndexOf('\0') >= 0 || value.IndexOf('\0') >= 0)
                    throw new ArgumentException("Environment cannot contain embedded NUL.");
                env.Append(key).Append('=').Append(value).Append('\0');
            }
            env.Append('\0');
            if (keys.Count == 0) env.Append('\0');
            IntPtr environment = Marshal.StringToHGlobalUni(env.ToString());
            ProcessInformation pi = new ProcessInformation();
            try
            {
                var startup = new StartupInfo();
                startup.cb = Marshal.SizeOf(typeof(StartupInfo));
                startup.flags = 1; // STARTF_USESHOWWINDOW
                startup.showWindow = 0; // SW_HIDE
                var command = new StringBuilder("\"" + info.FileName + "\"" +
                    (String.IsNullOrEmpty(info.Arguments) ? "" : " " + info.Arguments));
                const uint CreateUnicodeEnvironment = 0x00000400;
                const uint CreateNoWindow = 0x08000000;
                if (!CreateProcessW(info.FileName, command, IntPtr.Zero, IntPtr.Zero,
                    false, CreateUnicodeEnvironment | CreateNoWindow, environment,
                    info.WorkingDirectory, ref startup, out pi))
                    throw new Win32Exception(Marshal.GetLastWin32Error(), "No-inherit CreateProcessW failed.");
                try
                {
                    Process owned = Process.GetProcessById(pi.processId);
                    // Cache the .NET-owned handle while the native launch handle
                    // still exists, so HasExited/WaitForExit/ExitCode remain usable.
                    try { IntPtr attachedHandle = owned.Handle; return owned; }
                    catch { owned.Dispose(); throw; }
                }
                catch (Exception ex)
                {
                    // Terminate the owned child if .NET process attachment fails.
                    bool terminated = TerminateProcess(pi.process, 125);
                    uint waited = terminated ? WaitForSingleObject(pi.process, 5000) : UInt32.MaxValue;
                    throw new InvalidOperationException("Cannot attach to owned child PID=" + pi.processId +
                        "; cleanup_terminated=" + terminated + "; cleanup_wait=" + waited, ex);
                }
            }
            finally
            {
                if (pi.thread != IntPtr.Zero) CloseHandle(pi.thread);
                if (pi.process != IntPtr.Zero) CloseHandle(pi.process);
                Marshal.FreeHGlobal(environment);
            }
        }
    }
}
