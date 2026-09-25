// Read-only Windows host diagnostics for the outer child-wait loop.
// Sample about once per second, retain results in memory, and persist on cleanup.
// The caller supplies only PIDs verified as its MATLAB or CopterSim processes.
// GetProcessIoCounters includes file, network, device and other process IO.
// System CPU uses delta(kernel + user - idle)/delta(kernel + user).
// On systems with more than 64 processors, CPU data covers the caller's group.
// API references:
// https://learn.microsoft.com/windows/win32/api/processthreadsapi/nf-processthreadsapi-getsystemtimes
// https://learn.microsoft.com/windows/win32/api/winbase/nf-winbase-getprocessiocounters
// https://learn.microsoft.com/windows/win32/api/sysinfoapi/nf-sysinfoapi-globalmemorystatusex
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.Runtime.InteropServices;

namespace GPENMPC.HostDiagnostics
{
    public sealed class HostLoadSnapshot
    {
        public long Sequence { get; set; }
        public string Utc { get; set; }
        public long StopwatchTicks { get; set; }
        public long StopwatchFrequency { get; set; }
        public double SampleDurationMs { get; set; }
        public string Use { get { return "DIAGNOSTIC_ONLY_NO_REJECTION_OR_RESOURCE_HOLD"; } }
        public string SystemCpuScope { get { return "GetSystemTimes: all CPUs if <=64, otherwise calling processor group"; } }
        public int LogicalProcessorsReportedByRuntime { get; set; }
        public bool SystemTimesValid { get; set; }
        public ulong? SystemIdle100ns { get; set; }
        public ulong? SystemKernelIncludingIdle100ns { get; set; }
        public ulong? SystemUser100ns { get; set; }
        public double? SystemCpuBusyPercent { get; set; }
        public double? SystemCpuIntervalSeconds { get; set; }
        public bool MemoryValid { get; set; }
        public ulong? PhysicalTotalBytes { get; set; }
        public ulong? PhysicalAvailableBytes { get; set; }
        public uint? MemoryLoadPercent { get; set; }
        public HostProcessLoad[] Processes { get; set; }
        public string[] Diagnostics { get; set; }
        public int QueryHandlesOpened { get; set; }
        public int QueryHandlesCloseAttempted { get; set; }
        public int QueryHandlesClosed { get; set; }
        public int QueryHandlesUnconfirmedClosed { get; set; }
    }

    public sealed class HostProcessLoad
    {
        public int ProcessId { get; set; }
        public bool QueryHandleOpened { get; set; }
        public bool QueryHandleClosed { get; set; }
        public bool ProcessTimesValid { get; set; }
        public ulong? CreationFileTime100ns { get; set; }
        public ulong? ExitFileTime100ns { get; set; }
        public ulong? KernelCpu100ns { get; set; }
        public ulong? UserCpu100ns { get; set; }
        public bool? ExitObserved { get; set; }
        public bool IoCountersValid { get; set; }
        public ulong? ReadOperationCount { get; set; }
        public ulong? WriteOperationCount { get; set; }
        public ulong? OtherOperationCount { get; set; }
        public ulong? ReadTransferBytes { get; set; }
        public ulong? WriteTransferBytes { get; set; }
        public ulong? OtherTransferBytes { get; set; }
        public string IoScope { get { return "PROCESS_FILE_NETWORK_DEVICE_OTHER_IO_NOT_PHYSICAL_DISK"; } }
        public bool DeltaValid { get; set; }
        public double? IntervalSeconds { get; set; }
        public double? CpuCoreEquivalent { get; set; }
        public double? ReadBytesPerSecond { get; set; }
        public double? WriteBytesPerSecond { get; set; }
        public double? OtherBytesPerSecond { get; set; }
        public string[] Diagnostics { get; set; }
    }

    // Store previous counters for one outer-loop caller.
    // API samples are non-atomic; timestamps and duration record their observation interval.
    public sealed class HostLoadSampler
    {
        private long sequence;
        private ulong previousIdle, previousKernel, previousUser;
        private long previousSystemTick;
        private bool haveSystem;
        private Dictionary<int, PreviousProcess> previous = new Dictionary<int, PreviousProcess>();
        private sealed class PreviousProcess
        {
            public ulong Creation, Kernel, User;
            public IO_COUNTERS Io;
            public long Tick;
        }

        public HostLoadSnapshot Sample(int[] callerProvenOwnedProcessIds)
        {
            long start = Stopwatch.GetTimestamp();
            var row = new HostLoadSnapshot {
                Sequence = ++sequence, Utc = DateTime.UtcNow.ToString("o", CultureInfo.InvariantCulture),
                StopwatchTicks = start, StopwatchFrequency = Stopwatch.Frequency,
                LogicalProcessorsReportedByRuntime = Environment.ProcessorCount
            };
            var diagnostics = new List<string>();
            var processes = new List<HostProcessLoad>();
            var next = new Dictionary<int, PreviousProcess>();
            try
            {
                FILETIME idle, kernel, user;
                if (GetSystemTimes(out idle, out kernel, out user))
                {
                    ulong i = AsUInt64(idle), k = AsUInt64(kernel), u = AsUInt64(user);
                    row.SystemTimesValid = true;
                    row.SystemIdle100ns = i; row.SystemKernelIncludingIdle100ns = k; row.SystemUser100ns = u;
                    if (haveSystem && i >= previousIdle && k >= previousKernel && u >= previousUser)
                    {
                        double total = (double)(k - previousKernel) + (u - previousUser);
                        double idleDelta = i - previousIdle;
                        if (total > 0 && idleDelta <= total)
                        {
                            row.SystemCpuBusyPercent = 100.0 * (total - idleDelta) / total;
                            row.SystemCpuIntervalSeconds = (start - previousSystemTick) / (double)Stopwatch.Frequency;
                        }
                        else diagnostics.Add("SYSTEM_CPU_DELTA_UNAVAILABLE_OR_INCONSISTENT");
                    }
                    else diagnostics.Add(haveSystem ? "SYSTEM_COUNTER_REGRESSION_NO_DELTA" : "FIRST_SYSTEM_SAMPLE_NO_CPU_DELTA");
                    previousIdle = i; previousKernel = k; previousUser = u; previousSystemTick = start; haveSystem = true;
                }
                else { diagnostics.Add(Error("GetSystemTimes")); haveSystem = false; }
            }
            catch (Exception e) { diagnostics.Add("SYSTEM_QUERY_EXCEPTION: " + e.GetType().Name + ": " + e.Message); haveSystem = false; }
            try
            {
                var memory = new MEMORYSTATUSEX();
                memory.dwLength = (uint)Marshal.SizeOf(typeof(MEMORYSTATUSEX));
                if (GlobalMemoryStatusEx(ref memory))
                {
                    row.MemoryValid = true; row.PhysicalTotalBytes = memory.ullTotalPhys;
                    row.PhysicalAvailableBytes = memory.ullAvailPhys; row.MemoryLoadPercent = memory.dwMemoryLoad;
                }
                else diagnostics.Add(Error("GlobalMemoryStatusEx"));
            }
            catch (Exception e) { diagnostics.Add("MEMORY_QUERY_EXCEPTION: " + e.GetType().Name + ": " + e.Message); }
            var seen = new HashSet<int>();
            foreach (int pid in callerProvenOwnedProcessIds ?? new int[0])
            {
                if (!seen.Add(pid)) { diagnostics.Add("DUPLICATE_PID_SKIPPED: " + pid); continue; }
                processes.Add(SampleProcess(pid, row, next));
            }
            previous = next; // Reset delta history for a failed or removed PID.
            row.Processes = processes.ToArray(); row.Diagnostics = diagnostics.ToArray();
            row.QueryHandlesUnconfirmedClosed = row.QueryHandlesOpened - row.QueryHandlesClosed;
            row.SampleDurationMs = (Stopwatch.GetTimestamp() - start) * 1000.0 / Stopwatch.Frequency;
            return row;
        }

        private HostProcessLoad SampleProcess(int pid, HostLoadSnapshot sample, Dictionary<int, PreviousProcess> next)
        {
            var row = new HostProcessLoad { ProcessId = pid };
            var diagnostics = new List<string>();
            IntPtr handle = IntPtr.Zero;
            try
            {
                if (pid <= 0) { diagnostics.Add("INVALID_PID_NO_OPEN"); return row; }
                handle = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, false, (uint)pid);
                if (handle == IntPtr.Zero) { diagnostics.Add(Error("OpenProcess(query limited)")); return row; }
                row.QueryHandleOpened = true; sample.QueryHandlesOpened++;
                FILETIME creation, exit, kernel, user;
                if (GetProcessTimes(handle, out creation, out exit, out kernel, out user))
                {
                    row.ProcessTimesValid = true; row.CreationFileTime100ns = AsUInt64(creation);
                    row.ExitFileTime100ns = AsUInt64(exit); row.ExitObserved = row.ExitFileTime100ns != 0;
                    row.KernelCpu100ns = AsUInt64(kernel); row.UserCpu100ns = AsUInt64(user);
                }
                else diagnostics.Add(Error("GetProcessTimes"));
                IO_COUNTERS io;
                if (GetProcessIoCounters(handle, out io))
                {
                    row.IoCountersValid = true;
                    row.ReadOperationCount = io.ReadOperationCount; row.WriteOperationCount = io.WriteOperationCount;
                    row.OtherOperationCount = io.OtherOperationCount; row.ReadTransferBytes = io.ReadTransferCount;
                    row.WriteTransferBytes = io.WriteTransferCount; row.OtherTransferBytes = io.OtherTransferCount;
                    long tick = Stopwatch.GetTimestamp();
                    if (row.ProcessTimesValid)
                    {
                        var current = new PreviousProcess { Creation = row.CreationFileTime100ns.Value,
                            Kernel = row.KernelCpu100ns.Value, User = row.UserCpu100ns.Value, Io = io, Tick = tick };
                        PreviousProcess old;
                        if (previous.TryGetValue(pid, out old))
                        {
                            double seconds = (tick - old.Tick) / (double)Stopwatch.Frequency;
                            if (current.Creation == old.Creation && seconds > 0 && current.Kernel >= old.Kernel &&
                                current.User >= old.User && IoNondecreasing(io, old.Io))
                            {
                                row.DeltaValid = true; row.IntervalSeconds = seconds;
                                row.CpuCoreEquivalent = ((double)(current.Kernel - old.Kernel) + (current.User - old.User)) / (1e7 * seconds);
                                row.ReadBytesPerSecond = (io.ReadTransferCount - old.Io.ReadTransferCount) / seconds;
                                row.WriteBytesPerSecond = (io.WriteTransferCount - old.Io.WriteTransferCount) / seconds;
                                row.OtherBytesPerSecond = (io.OtherTransferCount - old.Io.OtherTransferCount) / seconds;
                            }
                            else diagnostics.Add("PID_REUSED_OR_COUNTER_REGRESSION_NO_DELTA");
                        }
                        else diagnostics.Add("FIRST_PROCESS_SAMPLE_NO_DELTA");
                        next[pid] = current;
                    }
                }
                else diagnostics.Add(Error("GetProcessIoCounters"));
            }
            catch (Exception e) { diagnostics.Add("PROCESS_QUERY_EXCEPTION: " + e.GetType().Name + ": " + e.Message); }
            finally
            {
                if (handle != IntPtr.Zero)
                {
                    sample.QueryHandlesCloseAttempted++;
                    if (CloseHandle(handle)) { row.QueryHandleClosed = true; sample.QueryHandlesClosed++; }
                    else diagnostics.Add(Error("CloseHandle(query)"));
                }
                row.Diagnostics = diagnostics.ToArray();
            }
            return row;
        }

        private static bool IoNondecreasing(IO_COUNTERS a, IO_COUNTERS b)
        {
            return a.ReadOperationCount >= b.ReadOperationCount && a.WriteOperationCount >= b.WriteOperationCount &&
                a.OtherOperationCount >= b.OtherOperationCount && a.ReadTransferCount >= b.ReadTransferCount &&
                a.WriteTransferCount >= b.WriteTransferCount && a.OtherTransferCount >= b.OtherTransferCount;
        }
        private static string Error(string api) { return api + " WIN32_ERROR=" + Marshal.GetLastWin32Error(); }
        private static ulong AsUInt64(FILETIME t) { return ((ulong)t.High << 32) | t.Low; }
        private const uint PROCESS_QUERY_LIMITED_INFORMATION = 0x1000;
        [StructLayout(LayoutKind.Sequential)] private struct FILETIME { public uint Low, High; }
        [StructLayout(LayoutKind.Sequential)] private struct IO_COUNTERS
        {
            public ulong ReadOperationCount, WriteOperationCount, OtherOperationCount;
            public ulong ReadTransferCount, WriteTransferCount, OtherTransferCount;
        }
        [StructLayout(LayoutKind.Sequential)] private struct MEMORYSTATUSEX
        {
            public uint dwLength, dwMemoryLoad;
            public ulong ullTotalPhys, ullAvailPhys, ullTotalPageFile, ullAvailPageFile;
            public ulong ullTotalVirtual, ullAvailVirtual, ullAvailExtendedVirtual;
        }
        [DllImport("kernel32.dll", SetLastError = true)] [return: MarshalAs(UnmanagedType.Bool)]
        private static extern bool GetSystemTimes(out FILETIME idle, out FILETIME kernel, out FILETIME user);
        [DllImport("kernel32.dll", SetLastError = true)] [return: MarshalAs(UnmanagedType.Bool)]
        private static extern bool GlobalMemoryStatusEx(ref MEMORYSTATUSEX info);
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern IntPtr OpenProcess(uint access, [MarshalAs(UnmanagedType.Bool)] bool inherit, uint pid);
        [DllImport("kernel32.dll", SetLastError = true)] [return: MarshalAs(UnmanagedType.Bool)]
        private static extern bool GetProcessTimes(IntPtr process, out FILETIME creation, out FILETIME exit, out FILETIME kernel, out FILETIME user);
        [DllImport("kernel32.dll", SetLastError = true)] [return: MarshalAs(UnmanagedType.Bool)]
        private static extern bool GetProcessIoCounters(IntPtr process, out IO_COUNTERS counters);
        [DllImport("kernel32.dll", SetLastError = true)] [return: MarshalAs(UnmanagedType.Bool)]
        private static extern bool CloseHandle(IntPtr handle);
    }
}
