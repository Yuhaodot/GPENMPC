using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace GPENMPC.HostDiagnostics
{
    // Observe ReqCopterSim.GetUe4EKFInit packets: 12-byte native 3i fields
    // checksum, CopterID and initCode. Interpret initCode 0/1 and retain checksum.
    public sealed class OwnedCopterInitObserver : IDisposable
    {
        public sealed class Packet
        {
            public long sequence;
            public string received_utc;
            public long received_qpc;
            public double elapsed_s;
            public string remote;
            public int length;
            public string raw_base64;
            public bool parsed_3i;
            public int checksum_raw;
            public int copter_id;
            public int init_code;
            public bool expected_vehicle;
            public string interpretation;
        }
        public sealed class Evidence
        {
            public string schema = "OWNED_COPTER_INIT_RECEIVE_ONLY_V1";
            public string source_protocol = "ReqCopterSim.py GetUe4EKFInit: 224.0.0.10:20009, little-endian 3i";
            public string checksum_policy = "RAW_ONLY__OFFICIAL_RECEIVER_DOES_NOT_VALIDATE_CONSTANT";
            public int local_port;
            public string multicast_group;
            public int expected_vehicle;
            public int capacity;
            public long qpc_frequency;
            public long received_datagrams;
            public long stored_datagrams;
            public long overflow_dropped;
            public long retained_raw_bytes;
            public long max_retained_raw_bytes;
            public long errors_dropped;
            public bool evidence_complete;
            public bool socket_closed;
            public bool thread_exited;
            public bool receive_only = true;
            public int send_count = 0;
            public int COM_open = 0;
            public string[] errors;
            public Packet[] packets;
        }

        private readonly object gate = new object();
        private readonly List<Packet> packets = new List<Packet>();
        private readonly List<string> errors = new List<string>();
        private readonly Socket socket;
        private readonly Thread worker;
        private readonly int capacity;
        private readonly int expectedVehicle;
        private readonly long startQpc;
        private long received;
        private long overflow;
        private long retainedRawBytes;
        private const long MaxRetainedRawBytes = 16 * 1024 * 1024;
        private long errorsDropped;
        private volatile bool stopping;
        private volatile bool socketClosed;
        private volatile bool threadExited;
        private int disposed;
        private readonly string multicastGroup;
        public int Port { get; private set; }

        // port=0 is only for isolated localhost fixture allocation.
        public static OwnedCopterInitObserver Start(int port, int expectedVehicle, int capacity)
        {
            return new OwnedCopterInitObserver(port, expectedVehicle, capacity);
        }
        private OwnedCopterInitObserver(int port, int expectedVehicle, int capacity)
        {
            if (port < 0 || port > 65535 || expectedVehicle <= 0 || capacity <= 0 || capacity > 65536)
                throw new ArgumentOutOfRangeException("port/vehicle/capacity");
            if (!BitConverter.IsLittleEndian)
                throw new PlatformNotSupportedException("Official Windows 3i observer expects little endian.");
            this.capacity = capacity;
            this.expectedVehicle = expectedVehicle;
            multicastGroup = port == 0 ? "" : "224.0.0.10";
            startQpc = Stopwatch.GetTimestamp();
            socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            try
            {
                socket.ExclusiveAddressUse = true;
                socket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, false);
                socket.ReceiveTimeout = 100;
                socket.Bind(new IPEndPoint(port == 0 ? IPAddress.Loopback : IPAddress.Any, port));
                Port = ((IPEndPoint)socket.LocalEndPoint).Port;
                if (multicastGroup.Length != 0)
                    socket.SetSocketOption(SocketOptionLevel.IP, SocketOptionName.AddMembership,
                        new MulticastOption(IPAddress.Parse(multicastGroup), IPAddress.Any));
                worker = new Thread(ReceiveLoop);
                worker.IsBackground = true;
                worker.Name = "GPENMPC-CopterInit-ReceiveOnly";
                worker.Start();
            }
            catch { socket.Dispose(); socketClosed = true; throw; }
        }
        private void ReceiveLoop()
        {
            byte[] buffer = new byte[65535];
            try
            {
                while (!stopping)
                {
                    EndPoint remote = new IPEndPoint(IPAddress.Any, 0);
                    int n;
                    try { n = socket.ReceiveFrom(buffer, ref remote); }
                    catch (SocketException ex)
                    {
                        if (stopping) break;
                        if (ex.SocketErrorCode == SocketError.TimedOut) continue;
                        RecordError(ex.SocketErrorCode + ": " + ex.Message);
                        break;
                    }
                    catch (ObjectDisposedException) { if (stopping) break; throw; }
                    long qpc = Stopwatch.GetTimestamp();
                    string utc = DateTime.UtcNow.ToString("o");
                    lock (gate)
                    {
                        received++;
                        if (packets.Count >= capacity || retainedRawBytes + n > MaxRetainedRawBytes) { overflow++; continue; }
                        Packet p = new Packet();
                        p.sequence = received; p.received_qpc = qpc; p.received_utc = utc;
                        p.elapsed_s = (qpc - startQpc) / (double)Stopwatch.Frequency;
                        p.remote = remote.ToString(); p.length = n;
                        p.raw_base64 = Convert.ToBase64String(buffer, 0, n);
                        p.parsed_3i = n == 12;
                        p.interpretation = "UNINTERPRETED_BAD_LENGTH";
                        if (p.parsed_3i)
                        {
                            p.checksum_raw = BitConverter.ToInt32(buffer, 0);
                            p.copter_id = BitConverter.ToInt32(buffer, 4);
                            p.init_code = BitConverter.ToInt32(buffer, 8);
                            p.expected_vehicle = p.copter_id == expectedVehicle;
                            p.interpretation = !p.expected_vehicle ? "OTHER_VEHICLE_RAW_ONLY" :
                                p.init_code == 0 ? "VENDOR_INITIALIZATION_NOT_FINISHED" :
                                p.init_code == 1 ? "VENDOR_GPS3D_FIXED_REPORTED" : "UNKNOWN_INIT_CODE_RAW_ONLY";
                        }
                        packets.Add(p);
                        retainedRawBytes += n;
                    }
                }
            }
            catch (Exception ex) { RecordError(ex.GetType().Name + ": " + ex.Message); }
            finally { CloseSocket(); threadExited = true; }
        }
        private void RecordError(string message)
        {
            lock (gate) { if (errors.Count < 16) errors.Add(message); else errorsDropped++; }
        }
        private void CloseSocket()
        {
            try { socket.Dispose(); socketClosed = true; }
            catch (Exception ex) { RecordError("Close: " + ex.Message); }
        }
        public Evidence Snapshot()
        {
            lock (gate)
            {
                Evidence e = new Evidence();
                e.local_port = Port; e.multicast_group = multicastGroup;
                e.expected_vehicle = expectedVehicle; e.capacity = capacity;
                e.qpc_frequency = Stopwatch.Frequency; e.received_datagrams = received;
                e.stored_datagrams = packets.Count; e.overflow_dropped = overflow;
                e.retained_raw_bytes = retainedRawBytes; e.max_retained_raw_bytes = MaxRetainedRawBytes;
                e.errors_dropped = errorsDropped;
                e.evidence_complete = overflow == 0 && errors.Count == 0;
                e.socket_closed = socketClosed; e.thread_exited = threadExited;
                e.errors = errors.ToArray(); e.packets = packets.ToArray();
                return e;
            }
        }
        public void Dispose()
        {
            if (Interlocked.Exchange(ref disposed, 1) != 0) return;
            stopping = true;
            CloseSocket();
            // Engineering thread cleanup bound, not a HIL performance/safety gate.
            if (!worker.Join(2000)) RecordError("Receive thread did not exit within 2000 ms after socket close.");
        }
    }
}
