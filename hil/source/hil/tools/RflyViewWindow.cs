using System;
using System.Diagnostics;
using System.IO;
using System.Runtime.InteropServices;
using System.Threading;

namespace GPENMPC.Display
{
    // Present the viewer on an explicit dashboard action.
    public static class RflyViewWindow
    {
        private static string OfficialChild
        {
            get
            {
                string root = Environment.GetEnvironmentVariable("GPENMPC_RFLY_ROOT");
                if (String.IsNullOrWhiteSpace(root) || !Directory.Exists(root))
                    throw new InvalidOperationException("Configure GPENMPC_RFLY_ROOT with the RflySim installation directory.");
                string child = Path.GetFullPath(Path.Combine(root, @"RflySim3D\RflySim3D\Binaries\Win64\RflySim3D.exe"));
                if (!File.Exists(child)) throw new InvalidOperationException("The installed RflySim3D renderer was not found.");
                return child;
            }
        }
        [StructLayout(LayoutKind.Sequential)]
        private struct Rect { public int Left, Top, Right, Bottom; }
        [StructLayout(LayoutKind.Sequential)]
        private struct MonitorInfo { public int Size; public Rect Monitor, Work; public uint Flags; }
        [DllImport("user32.dll")] private static extern bool IsWindow(IntPtr window);
        [DllImport("user32.dll")] private static extern bool IsWindowVisible(IntPtr window);
        [DllImport("user32.dll")] private static extern bool IsIconic(IntPtr window);
        [DllImport("user32.dll")] private static extern uint GetWindowThreadProcessId(IntPtr window, out uint pid);
        [DllImport("user32.dll")] private static extern bool ShowWindowAsync(IntPtr window, int command);
        [DllImport("user32.dll")] private static extern bool GetWindowRect(IntPtr window, out Rect rect);
        [DllImport("user32.dll")] private static extern IntPtr MonitorFromRect(ref Rect rect, uint flags);
        [DllImport("user32.dll", CharSet = CharSet.Unicode)] private static extern bool GetMonitorInfo(IntPtr monitor, ref MonitorInfo info);
        [DllImport("user32.dll")] private static extern bool SetWindowPos(IntPtr window, IntPtr after, int x, int y, int cx, int cy, uint flags);
        [DllImport("user32.dll")] private static extern bool SetForegroundWindow(IntPtr window);
        [DllImport("user32.dll")] private static extern IntPtr GetForegroundWindow();

        public sealed class Receipt
        {
            public bool Restored, Repositioned, Visible, Foreground;
            public int Pid;
            public long Window;
        }

        // Compute window placement.
        public static int[] CenterInWorkArea(int width, int height, int left, int top, int right, int bottom)
        {
            if (width <= 0 || height <= 0 || right <= left || bottom <= top)
                throw new ArgumentException("Window and monitor dimensions must be positive.");
            return new int[] { left + Math.Max(0, (right - left - width) / 2),
                top + Math.Max(0, (bottom - top - height) / 2) };
        }

        public static Receipt Present(int pid, long expectedHandle)
        {
            if (pid <= 0 || expectedHandle == 0) throw new ArgumentException("An existing renderer and window are required.");
            IntPtr window = new IntPtr(expectedHandle);
            using (Process process = Process.GetProcessById(pid))
            {
                process.Refresh();
                if (process.HasExited || !String.Equals(Path.GetFullPath(process.MainModule.FileName),
                    OfficialChild, StringComparison.OrdinalIgnoreCase))
                    throw new InvalidOperationException("The selected process is not the installed RflySim3D renderer.");
                uint owner;
                GetWindowThreadProcessId(window, out owner);
                if (!IsWindow(window) || owner != (uint)pid || process.MainWindowHandle != window)
                    throw new InvalidOperationException("The renderer window changed; no other window was modified.");
                Receipt result = new Receipt { Pid = pid, Window = expectedHandle };
                if (IsIconic(window)) { ShowWindowAsync(window, 9); result.Restored = true; }
                else if (!IsWindowVisible(window)) { ShowWindowAsync(window, 5); result.Restored = true; }
                Stopwatch wait = Stopwatch.StartNew();
                while (wait.ElapsedMilliseconds < 750 && (!IsWindowVisible(window) || IsIconic(window))) Thread.Sleep(25);
                Rect rect;
                if (!GetWindowRect(window, out rect)) throw new InvalidOperationException("Cannot read the renderer window position.");
                if (MonitorFromRect(ref rect, 0) == IntPtr.Zero)
                {
                    IntPtr monitor = MonitorFromRect(ref rect, 2); // nearest current monitor
                    MonitorInfo info = new MonitorInfo { Size = Marshal.SizeOf(typeof(MonitorInfo)) };
                    if (!GetMonitorInfo(monitor, ref info)) throw new InvalidOperationException("Cannot locate a visible monitor.");
                    int[] at = CenterInWorkArea(rect.Right - rect.Left, rect.Bottom - rect.Top,
                        info.Work.Left, info.Work.Top, info.Work.Right, info.Work.Bottom);
                    // Relocate while preserving size, topmost state and focus.
                    if (!SetWindowPos(window, IntPtr.Zero, at[0], at[1], 0, 0, 0x4015))
                        throw new InvalidOperationException("Cannot restore the renderer to a visible monitor.");
                    result.Repositioned = true;
                }
                result.Visible = IsWindowVisible(window) && !IsIconic(window);
                if (!result.Visible) throw new InvalidOperationException("The renderer did not restore its window.");
                // Respect Windows foreground-activation policy without repeated activation attempts.
                SetForegroundWindow(window);
                result.Foreground = GetForegroundWindow() == window;
                return result;
            }
        }
    }
}
