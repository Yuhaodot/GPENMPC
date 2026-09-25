// Contain the owned process tree in a Windows Job (Windows 8 or later).
// Create the child suspended, assign it to the Job, then resume it.
// Restrict inherited handles to stdin, stdout and stderr.
using System;
using System.Diagnostics;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

public sealed class OwnedMatlabJob : IDisposable
{
    private IntPtr job = IntPtr.Zero;
    private IntPtr process = IntPtr.Zero;
    private bool disposed;
    public int ProcessId { get; private set; }
    public bool AssignedBeforeResume { get; private set; }

    private OwnedMatlabJob() { }

    public static OwnedMatlabJob Start(string executable, string arguments,
        string workingDirectory, string stdoutPath, string stderrPath)
    {
        executable = Path.GetFullPath(executable);
        workingDirectory = Path.GetFullPath(workingDirectory);
        stdoutPath = Path.GetFullPath(stdoutPath);
        stderrPath = Path.GetFullPath(stderrPath);
        if (!File.Exists(executable) || !Directory.Exists(workingDirectory))
            throw new ArgumentException("Exact executable and working directory must exist.");
        if (executable.IndexOf('"') >= 0 || executable.IndexOf('\0') >= 0 ||
            (arguments ?? "").IndexOf('\0') >= 0)
            throw new ArgumentException("Invalid process command line.");
        if (String.Equals(stdoutPath, stderrPath, StringComparison.OrdinalIgnoreCase) ||
            File.Exists(stdoutPath) || File.Exists(stderrPath))
            throw new IOException("Distinct fresh stdout/stderr paths are required; no overwrite.");
        if (!Directory.Exists(Path.GetDirectoryName(stdoutPath)) ||
            !Directory.Exists(Path.GetDirectoryName(stderrPath)))
            throw new DirectoryNotFoundException("Log directories must already exist.");

        var owner = new OwnedMatlabJob();
        IntPtr input = IntPtr.Zero, output = IntPtr.Zero, error = IntPtr.Zero;
        IntPtr attributes = IntPtr.Zero, handleList = IntPtr.Zero;
        PROCESS_INFORMATION pi = new PROCESS_INFORMATION();
        bool processCreated = false;
        bool attributesInitialized = false;
        try
        {
            owner.job = CreateJobObjectW(IntPtr.Zero, null);
            RequireHandle(owner.job, "CreateJobObjectW");
            var limits = new JOBOBJECT_EXTENDED_LIMIT_INFORMATION();
            limits.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
            if (!SetInformationJobObject(owner.job, 9, ref limits,
                (uint)Marshal.SizeOf(typeof(JOBOBJECT_EXTENDED_LIMIT_INFORMATION))))
                Fail("SetInformationJobObject(KILL_ON_JOB_CLOSE)");

            var security = new SECURITY_ATTRIBUTES();
            security.nLength = Marshal.SizeOf(typeof(SECURITY_ATTRIBUTES));
            security.bInheritHandle = true;
            input = CreateFileW("NUL", GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE,
                ref security, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, IntPtr.Zero);
            RequireHandle(input, "CreateFileW(stdin NUL)");
            output = CreateFileW(stdoutPath, GENERIC_WRITE, FILE_SHARE_READ,
                ref security, CREATE_NEW, FILE_ATTRIBUTE_NORMAL, IntPtr.Zero);
            RequireHandle(output, "CreateFileW(fresh stdout)");
            error = CreateFileW(stderrPath, GENERIC_WRITE, FILE_SHARE_READ,
                ref security, CREATE_NEW, FILE_ATTRIBUTE_NORMAL, IntPtr.Zero);
            RequireHandle(error, "CreateFileW(fresh stderr)");

            IntPtr attributeBytes = IntPtr.Zero;
            InitializeProcThreadAttributeList(IntPtr.Zero, 1, 0, ref attributeBytes);
            if (attributeBytes == IntPtr.Zero) Fail("InitializeProcThreadAttributeList(size)");
            attributes = Marshal.AllocHGlobal(attributeBytes);
            if (!InitializeProcThreadAttributeList(attributes, 1, 0, ref attributeBytes))
                Fail("InitializeProcThreadAttributeList");
            attributesInitialized = true;
            handleList = Marshal.AllocHGlobal(3 * IntPtr.Size);
            Marshal.WriteIntPtr(handleList, 0, input);
            Marshal.WriteIntPtr(handleList, IntPtr.Size, output);
            Marshal.WriteIntPtr(handleList, 2 * IntPtr.Size, error);
            if (!UpdateProcThreadAttribute(attributes, 0,
                new IntPtr(PROC_THREAD_ATTRIBUTE_HANDLE_LIST), handleList,
                new IntPtr(3 * IntPtr.Size), IntPtr.Zero, IntPtr.Zero))
                Fail("UpdateProcThreadAttribute(HANDLE_LIST)");

            var startup = new STARTUPINFOEX();
            startup.StartupInfo.cb = Marshal.SizeOf(typeof(STARTUPINFOEX));
            startup.StartupInfo.dwFlags = STARTF_USESTDHANDLES;
            startup.StartupInfo.hStdInput = input;
            startup.StartupInfo.hStdOutput = output;
            startup.StartupInfo.hStdError = error;
            startup.lpAttributeList = attributes;
            var command = new StringBuilder("\"" + executable + "\"" +
                (String.IsNullOrWhiteSpace(arguments) ? "" : " " + arguments));
            if (!CreateProcessW(executable, command, IntPtr.Zero, IntPtr.Zero, true,
                CREATE_SUSPENDED | CREATE_NO_WINDOW | EXTENDED_STARTUPINFO_PRESENT,
                IntPtr.Zero, workingDirectory, ref startup, out pi))
                Fail("CreateProcessW(suspended)");
            processCreated = true;
            owner.process = pi.hProcess;
            owner.ProcessId = checked((int)pi.dwProcessId);
            if (!AssignProcessToJobObject(owner.job, owner.process))
                Fail("AssignProcessToJobObject(before resume)");
            owner.AssignedBeforeResume = true;
            if (ResumeThread(pi.hThread) == UInt32.MaxValue) Fail("ResumeThread");
            Close(ref pi.hThread);
            return owner;
        }
        catch
        {
            // Keep the child suspended if Job assignment fails.
            if (processCreated && owner.process != IntPtr.Zero)
            {
                TerminateProcess(owner.process, 253);
                WaitForSingleObject(owner.process, 5000);
            }
            owner.Dispose();
            throw;
        }
        finally
        {
            Close(ref pi.hThread);
            if (attributes != IntPtr.Zero)
            {
                if (attributesInitialized) DeleteProcThreadAttributeList(attributes);
                Marshal.FreeHGlobal(attributes);
            }
            if (handleList != IntPtr.Zero) Marshal.FreeHGlobal(handleList);
            Close(ref input); Close(ref output); Close(ref error);
        }
    }

    public int ActiveProcesses
    {
        get
        {
            RequireOpen();
            JOBOBJECT_BASIC_ACCOUNTING_INFORMATION accounting;
            if (!QueryInformationJobObject(job, 1, out accounting,
                (uint)Marshal.SizeOf(typeof(JOBOBJECT_BASIC_ACCOUNTING_INFORMATION)), IntPtr.Zero))
                Fail("QueryInformationJobObject(ActiveProcesses)");
            return checked((int)accounting.ActiveProcesses);
        }
    }

    public int ExitCode
    {
        get
        {
            RequireOpen();
            if (WaitForSingleObject(process, 0) != WAIT_OBJECT_0)
                throw new InvalidOperationException("Owned root process has not exited.");
            uint code;
            if (!GetExitCodeProcess(process, out code)) Fail("GetExitCodeProcess");
            return unchecked((int)code);
        }
    }

    public int[] ProcessIds
    {
        get
        {
            RequireOpen();
            int capacity = 16;
            for (int attempt = 0; attempt < 8; attempt++)
            {
                int size = checked(8 + capacity * IntPtr.Size);
                IntPtr buffer = Marshal.AllocHGlobal(size);
                try
                {
                    bool ok = QueryInformationJobObjectBuffer(job, 3, buffer, (uint)size, IntPtr.Zero);
                    int errorCode = ok ? 0 : Marshal.GetLastWin32Error();
                    if (!ok && errorCode != 234)
                        throw new System.ComponentModel.Win32Exception(errorCode, "QueryInformationJobObject(ProcessIdList)");
                    uint assigned = unchecked((uint)Marshal.ReadInt32(buffer, 0));
                    uint listed = unchecked((uint)Marshal.ReadInt32(buffer, 4));
                    if (ok && listed <= capacity)
                    {
                        var ids = new int[checked((int)listed)];
                        for (int k = 0; k < ids.Length; k++)
                            ids[k] = checked((int)Marshal.ReadIntPtr(buffer, 8 + k * IntPtr.Size).ToInt64());
                        return ids;
                    }
                    if (assigned > 65536)
                        throw new InvalidOperationException("Owned Job process count exceeds diagnostic allocation bound.");
                    capacity = Math.Max(checked(capacity * 2), checked((int)assigned + 16));
                }
                finally { Marshal.FreeHGlobal(buffer); }
            }
            throw new InvalidOperationException("Owned process list changed continuously; no stable PID snapshot.");
        }
    }

    public bool WaitForTreeExit(int milliseconds)
    {
        RequireOpen();
        if (milliseconds < 0) throw new ArgumentOutOfRangeException("milliseconds");
        var elapsed = Stopwatch.StartNew();
        while (true)
        {
            if (ActiveProcesses == 0) return true;
            long remaining = milliseconds - elapsed.ElapsedMilliseconds;
            if (remaining <= 0) return false;
            Thread.Sleep((int)Math.Min(25, remaining));
        }
    }

    public void Terminate(uint exitCode)
    {
        RequireOpen();
        if (ActiveProcesses > 0 && !TerminateJobObject(job, exitCode))
            Fail("TerminateJobObject(owned subtree only)");
    }

    public void Dispose()
    {
        if (disposed) return;
        // Closing this Job applies KILL_ON_JOB_CLOSE to its members only.
        Close(ref job);
        Close(ref process);
        disposed = true;
        GC.SuppressFinalize(this);
    }

    ~OwnedMatlabJob() { Dispose(); }
    private void RequireOpen()
    {
        if (disposed || job == IntPtr.Zero || process == IntPtr.Zero)
            throw new ObjectDisposedException("OwnedMatlabJob");
    }
    private static void RequireHandle(IntPtr h, string api)
    {
        if (h == IntPtr.Zero || h == new IntPtr(-1)) Fail(api);
    }
    private static void Fail(string api)
    {
        throw new System.ComponentModel.Win32Exception(Marshal.GetLastWin32Error(), api);
    }
    private static void Close(ref IntPtr h)
    {
        if (h != IntPtr.Zero && h != new IntPtr(-1)) CloseHandle(h);
        h = IntPtr.Zero;
    }

    private const uint GENERIC_READ = 0x80000000, GENERIC_WRITE = 0x40000000;
    private const uint FILE_SHARE_READ = 1, FILE_SHARE_WRITE = 2;
    private const uint CREATE_NEW = 1, OPEN_EXISTING = 3, FILE_ATTRIBUTE_NORMAL = 0x80;
    private const uint CREATE_SUSPENDED = 4, CREATE_NO_WINDOW = 0x08000000;
    private const uint EXTENDED_STARTUPINFO_PRESENT = 0x00080000;
    private const uint STARTF_USESTDHANDLES = 0x100;
    private const uint JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE = 0x2000;
    private const long PROC_THREAD_ATTRIBUTE_HANDLE_LIST = 0x00020002;
    private const uint WAIT_OBJECT_0 = 0;

    [StructLayout(LayoutKind.Sequential)]
    private struct SECURITY_ATTRIBUTES
    {
        public int nLength;
        public IntPtr lpSecurityDescriptor;
        [MarshalAs(UnmanagedType.Bool)] public bool bInheritHandle;
    }
    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
    private struct STARTUPINFO
    {
        public int cb;
        public string lpReserved, lpDesktop, lpTitle;
        public uint dwX, dwY, dwXSize, dwYSize, dwXCountChars, dwYCountChars;
        public uint dwFillAttribute, dwFlags;
        public ushort wShowWindow, cbReserved2;
        public IntPtr lpReserved2, hStdInput, hStdOutput, hStdError;
    }
    [StructLayout(LayoutKind.Sequential)]
    private struct STARTUPINFOEX
    {
        public STARTUPINFO StartupInfo;
        public IntPtr lpAttributeList;
    }
    [StructLayout(LayoutKind.Sequential)]
    private struct PROCESS_INFORMATION
    {
        public IntPtr hProcess, hThread;
        public uint dwProcessId, dwThreadId;
    }
    [StructLayout(LayoutKind.Sequential)]
    private struct JOBOBJECT_BASIC_LIMIT_INFORMATION
    {
        public long PerProcessUserTimeLimit, PerJobUserTimeLimit;
        public uint LimitFlags;
        public UIntPtr MinimumWorkingSetSize, MaximumWorkingSetSize;
        public uint ActiveProcessLimit;
        public UIntPtr Affinity;
        public uint PriorityClass, SchedulingClass;
    }
    [StructLayout(LayoutKind.Sequential)]
    private struct IO_COUNTERS
    {
        public ulong ReadOperationCount, WriteOperationCount, OtherOperationCount;
        public ulong ReadTransferCount, WriteTransferCount, OtherTransferCount;
    }
    [StructLayout(LayoutKind.Sequential)]
    private struct JOBOBJECT_EXTENDED_LIMIT_INFORMATION
    {
        public JOBOBJECT_BASIC_LIMIT_INFORMATION BasicLimitInformation;
        public IO_COUNTERS IoInfo;
        public UIntPtr ProcessMemoryLimit, JobMemoryLimit, PeakProcessMemoryUsed, PeakJobMemoryUsed;
    }
    [StructLayout(LayoutKind.Sequential)]
    private struct JOBOBJECT_BASIC_ACCOUNTING_INFORMATION
    {
        public long TotalUserTime, TotalKernelTime, ThisPeriodTotalUserTime, ThisPeriodTotalKernelTime;
        public uint TotalPageFaultCount, TotalProcesses, ActiveProcesses, TotalTerminatedProcesses;
    }
    [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
    private static extern IntPtr CreateJobObjectW(IntPtr attributes, string name);
    [DllImport("kernel32.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool SetInformationJobObject(IntPtr job, int infoClass,
        ref JOBOBJECT_EXTENDED_LIMIT_INFORMATION info, uint size);
    [DllImport("kernel32.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool QueryInformationJobObject(IntPtr job, int infoClass,
        out JOBOBJECT_BASIC_ACCOUNTING_INFORMATION info, uint size, IntPtr returned);
    [DllImport("kernel32.dll", EntryPoint = "QueryInformationJobObject", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool QueryInformationJobObjectBuffer(IntPtr job, int infoClass,
        IntPtr info, uint size, IntPtr returned);
    [DllImport("kernel32.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool AssignProcessToJobObject(IntPtr job, IntPtr process);
    [DllImport("kernel32.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool TerminateJobObject(IntPtr job, uint exitCode);
    [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
    private static extern IntPtr CreateFileW(string name, uint access, uint share,
        ref SECURITY_ATTRIBUTES security, uint disposition, uint flags, IntPtr template);
    [DllImport("kernel32.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool InitializeProcThreadAttributeList(IntPtr attributes,
        int count, uint flags, ref IntPtr bytes);
    [DllImport("kernel32.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool UpdateProcThreadAttribute(IntPtr attributes, uint flags,
        IntPtr attribute, IntPtr value, IntPtr size, IntPtr previous, IntPtr returned);
    [DllImport("kernel32.dll")]
    private static extern void DeleteProcThreadAttributeList(IntPtr attributes);
    [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool CreateProcessW(string application, StringBuilder command,
        IntPtr processSecurity, IntPtr threadSecurity, bool inheritHandles, uint flags,
        IntPtr environment, string directory, ref STARTUPINFOEX startup,
        out PROCESS_INFORMATION processInfo);
    [DllImport("kernel32.dll", SetLastError = true)]
    private static extern uint ResumeThread(IntPtr thread);
    [DllImport("kernel32.dll", SetLastError = true)]
    private static extern uint WaitForSingleObject(IntPtr handle, uint milliseconds);
    [DllImport("kernel32.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool GetExitCodeProcess(IntPtr process, out uint code);
    [DllImport("kernel32.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool TerminateProcess(IntPtr process, uint code);
    [DllImport("kernel32.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool CloseHandle(IntPtr handle);
}
