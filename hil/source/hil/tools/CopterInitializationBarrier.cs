using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using System.Text.RegularExpressions;

namespace GPENMPC.HostDiagnostics
{
    // Parse vendor lifecycle text to determine readiness for a disarmed observer.
    // This barrier does not evaluate PX4 arming eligibility or reset adapter faults.
    public sealed class CopterInitializationBarrier
    {
        public sealed class Options
        {
            public string source_log_identity;
            public int expected_process_id;
            public bool log_created_exclusively_for_this_launch;
            public bool log_empty_before_launch;
            public string expected_dll_path;
            public string observed_dll_path;
            public string expected_dll_sha256;
            public string observed_dll_sha256;
        }
        public sealed class Line
        {
            public int sequence;
            public string raw;
            public double process_elapsed_s;
            public int process_id;
            public int thread_id;
            public string severity;
            public string message;
            public string interpretation;
        }
        public sealed class Evidence
        {
            public string schema = "COPTER_VENDOR_INITIALIZATION_BARRIER_TEXT_V1";
            public string status;
            public bool can_start_disarmed_observer;
            public bool flight_admission = false;
            public bool requires_independent_health = true;
            public bool requires_new_disarmed_observer = true;
            public bool must_not_reset_existing_adapter = true;
            public int initialization_prefix_observation_credit_s = 0;
            public double subsequent_complete_observation_s = 45.0;
            public double initialization_liveness_bound_s = 60.0;
            public string liveness_bound_provenance = "HOST_SESSION_ENGINEERING_LIVENESS_CANDIDATE_20S_INITIALIZATION_15S_RECONNECT_40S_READY_PLUS_20S_MARGIN";
            public int maximum_vendor_initialization_reboots = 1;
            public bool external_fresh_log_and_dll_proof_accepted;
            public Options external_proof;
            public string first_reject;
            public int first_reject_line;
            public double first_reject_wall_s;
            public int vendor_reboot_count;
            public int model_stop_count;
            public int sim_start_count;
            public int ready_report_count;
            public int invalidated_ready_reports;
            public int last_boundary_line;
            public int last_sim_start_line;
            public int last_ready_line;
            public double last_ready_process_s;
            public double current_wall_s;
            public double last_process_s;
            public string raw_prefix;
            public string pending_partial_line;
            public Line[] lines;
            public Line[] health_notes;
            public int uninterpreted_line_count;
            public int COM_open = 0;
            public int UDP_open = 0;
            public int board_actions = 0;
            public int parameter_writes = 0;
            public int model_actions = 0;
            public int arm_mode_task_commands = 0;
        }

        private static readonly Regex Header = new Regex(
            @"^\[\s*(?<time>\d+(?:\.\d+)?)\s+(?<level>\w+)\s+pid=(?<pid>\d+)\s+tid=(?<tid>\d+)\]\s*(?<body>.*)$",
            RegexOptions.CultureInvariant);
        private static readonly Regex ComTarget = new Regex(@"^Connect to\s+""COM3""\s+successful$", RegexOptions.CultureInvariant);
        private static readonly Regex Sha = new Regex(@"^[0-9a-fA-F]{64}$", RegexOptions.CultureInvariant);
        private readonly Options options;
        private readonly StringBuilder raw = new StringBuilder();
        private readonly StringBuilder pending = new StringBuilder();
        private readonly List<Line> lines = new List<Line>();
        private readonly List<Line> notes = new List<Line>();
        private bool proofAccepted;
        private string reject = "";
        private int rejectLine;
        private double rejectWall;
        private double wall = -1;
        private double lastTime = -1;
        private int reboots;
        private int stops;
        private int starts;
        private int readyCount;
        private int invalidatedReady;
        private int boundary;
        private int dllLine;
        private int comLine;
        private int startLine;
        private int readyLine;
        private double readyTime;
        private int unknown;
        private bool sensorApi;
        private bool gpsApi;
        private bool vehicleApi;

        public CopterInitializationBarrier(Options input)
        {
            // Copy identity fields to isolate accepted state from caller mutation.
            options = input == null ? new Options() : new Options {
                source_log_identity = input.source_log_identity,
                expected_process_id = input.expected_process_id,
                log_created_exclusively_for_this_launch = input.log_created_exclusively_for_this_launch,
                log_empty_before_launch = input.log_empty_before_launch,
                expected_dll_path = input.expected_dll_path,
                observed_dll_path = input.observed_dll_path,
                expected_dll_sha256 = input.expected_dll_sha256,
                observed_dll_sha256 = input.observed_dll_sha256
            };
            if (input == null || String.IsNullOrWhiteSpace(options.source_log_identity) || options.expected_process_id <= 0)
                Reject("MISSING_UNIQUE_LOG_PROCESS_IDENTITY", 0);
            else if (!options.log_created_exclusively_for_this_launch || !options.log_empty_before_launch)
                Reject("LOG_NOT_PROVEN_FRESH_FOR_THIS_LAUNCH", 0);
            else if (String.IsNullOrWhiteSpace(options.expected_dll_path) ||
                !String.Equals(options.expected_dll_path, options.observed_dll_path, StringComparison.OrdinalIgnoreCase) ||
                !Sha.IsMatch(options.expected_dll_sha256 ?? "") || !Sha.IsMatch(options.observed_dll_sha256 ?? "") ||
                !String.Equals(options.expected_dll_sha256, options.observed_dll_sha256, StringComparison.OrdinalIgnoreCase))
                Reject("EXTERNAL_DLL_IDENTITY_PROOF_MISMATCH", 0);
            else proofAccepted = true;
        }

        public void Append(string textChunk, double elapsedWallSeconds)
        {
            UpdateWall(elapsedWallSeconds);
            if (textChunk == null) { Reject("NULL_LOG_CHUNK", lines.Count + 1); return; }
            // The caller must retain its original stderr log. This pure classifier
            // additionally preserves every supplied character, including partial lines.
            raw.Append(textChunk); pending.Append(textChunk);
            int newline;
            while ((newline = IndexOfNewline(pending)) >= 0)
            {
                string original = pending.ToString(0, newline + 1);
                pending.Remove(0, newline + 1);
                Parse(original);
            }
            CheckDeadline();
        }

        public Evidence Snapshot(double elapsedWallSeconds)
        {
            UpdateWall(elapsedWallSeconds); CheckDeadline();
            bool ready = reject.Length == 0 && proofAccepted && HasCompleteReadyPrefix();
            return new Evidence {
                status = reject.Length != 0 ? "REJECTED_VENDOR_INITIALIZATION" : ready ?
                    "READY_FOR_NEW_DISARMED_OBSERVER_ONLY" : "WAITING_VENDOR_INITIALIZATION",
                can_start_disarmed_observer = ready,
                external_fresh_log_and_dll_proof_accepted = proofAccepted,
                external_proof = new Options {
                    source_log_identity=options.source_log_identity, expected_process_id=options.expected_process_id,
                    log_created_exclusively_for_this_launch=options.log_created_exclusively_for_this_launch,
                    log_empty_before_launch=options.log_empty_before_launch, expected_dll_path=options.expected_dll_path,
                    observed_dll_path=options.observed_dll_path, expected_dll_sha256=options.expected_dll_sha256,
                    observed_dll_sha256=options.observed_dll_sha256 },
                first_reject=reject, first_reject_line=rejectLine, first_reject_wall_s=rejectWall,
                vendor_reboot_count=reboots, model_stop_count=stops, sim_start_count=starts,
                ready_report_count=readyCount, invalidated_ready_reports=invalidatedReady,
                last_boundary_line=boundary, last_sim_start_line=startLine,
                last_ready_line=readyLine, last_ready_process_s=readyTime,
                current_wall_s=wall, last_process_s=lastTime,
                raw_prefix=raw.ToString(), pending_partial_line=pending.ToString(),
                lines=lines.ToArray(), health_notes=notes.ToArray(), uninterpreted_line_count=unknown
            };
        }

        private static int IndexOfNewline(StringBuilder b)
        { for (int i=0; i<b.Length; i++) if (b[i]=='\n') return i; return -1; }
        private void UpdateWall(double value)
        {
            if (Double.IsNaN(value) || Double.IsInfinity(value) || value < 0)
            { Reject("HOST_ELAPSED_CLOCK_NONFINITE_OR_NEGATIVE", lines.Count); return; }
            if (wall >= 0 && value < wall)
            { Reject("HOST_ELAPSED_CLOCK_REVERSED", lines.Count); return; }
            wall=value;
        }
        private void CheckDeadline()
        {
            if (wall >= 60.0 && !HasCompleteReadyPrefix())
                Reject("VENDOR_INITIALIZATION_LIVENESS_TIMEOUT_60S", lines.Count);
        }
        private bool HasCompleteReadyPrefix()
        {
            // Wait for a complete line before interpreting model stop, reboot or fatal status.
            return pending.Length == 0 && readyLine > startLine && startLine > boundary;
        }
        private void Reject(string cause, int line)
        {
            if (reject.Length != 0) return;
            reject=cause;rejectLine=line;rejectWall=Math.Max(wall,0);
        }
        private void Boundary(int line)
        {
            if (readyLine != 0) invalidatedReady++;
            boundary=line;dllLine=0;comLine=0;startLine=0;readyLine=0;readyTime=0;
            sensorApi=false;gpsApi=false;vehicleApi=false;
        }
        private void Parse(string original)
        {
            string value=original.TrimEnd('\r','\n');
            Line e=new Line {sequence=lines.Count+1,raw=original,message=value,interpretation="UNINTERPRETED"};
            lines.Add(e);
            Match match=Header.Match(value);
            if (!match.Success) { e.interpretation="MALFORMED_TIMESTAMPED_LOG_LINE";Reject(e.interpretation,e.sequence);return; }
            double time; int pid,tid;
            if (!Double.TryParse(match.Groups["time"].Value,NumberStyles.AllowDecimalPoint,CultureInfo.InvariantCulture,out time) ||
                !Int32.TryParse(match.Groups["pid"].Value,out pid) || !Int32.TryParse(match.Groups["tid"].Value,out tid))
            {e.interpretation="LOG_HEADER_VALUE_INVALID";Reject(e.interpretation,e.sequence);return;}
            e.process_elapsed_s=time;e.process_id=pid;e.thread_id=tid;
            e.severity=match.Groups["level"].Value;e.message=match.Groups["body"].Value.Trim();
            if (pid != options.expected_process_id) Reject("PROCESS_IDENTITY_CHANGED_OR_STALE_LOG",e.sequence);
            if (e.sequence==1 && time!=0.0) Reject("FRESH_LOG_ZERO_ORIGIN_MISSING",e.sequence);
            if (lastTime>=0 && time<lastTime) Reject("VENDOR_LOG_CLOCK_REVERSED",e.sequence);
            lastTime=time;
            string m=e.message;
            // Vendor reset boundaries invalidate readiness while preserving raw history.
            if (m=="model Stop!") {stops++;Boundary(e.sequence);e.interpretation="MODEL_STOP_BOUNDARY";return;}
            if (m=="Send reboot Commands.")
            {reboots++;Boundary(e.sequence);e.interpretation="VENDOR_INITIALIZATION_REBOOT";
                if(reboots>1)Reject("SECOND_VENDOR_INITIALIZATION_REBOOT",e.sequence);return;}
            if (m=="DLL&FUN Loaded Successfully!")
            {dllLine=e.sequence;sensorApi=false;gpsApi=false;vehicleApi=false;e.interpretation="DLL_LOAD_REPORTED_EXTERNAL_IDENTITY_REQUIRED";return;}
            if (m=="DlloutHILSensor30dFUN true!") {sensorApi=true;e.interpretation="REQUIRED_SENSOR_EXPORT_PRESENT";return;}
            if (m=="DlloutHILGPS30dFUN true!") {gpsApi=true;e.interpretation="REQUIRED_GPS_EXPORT_PRESENT";return;}
            if (m=="DlloutVehileInfo60dFUN true!") {vehicleApi=true;e.interpretation="REQUIRED_VEHICLE_EXPORT_PRESENT";return;}
            if (m=="DlloutHILSensor30dFUN false!" || m=="DlloutHILGPS30dFUN false!" || m=="DlloutVehileInfo60dFUN false!")
            {e.interpretation="REQUIRED_DLL_EXPORT_ABSENT";Reject(e.interpretation,e.sequence);return;}
            if (ComTarget.IsMatch(m)) {comLine=e.sequence;e.interpretation="EXACT_COM3_CONNECT_SUCCESS_REPORTED";return;}
            if (m=="Sim Start!")
            {
                starts++;startLine=e.sequence;readyLine=0;readyTime=0;e.interpretation="SIM_START";
                if(dllLine<=boundary||comLine<=boundary||!sensorApi||!gpsApi||!vehicleApi)
                    Reject("SIM_START_WITHOUT_FRESH_DLL_EXPORT_AND_COM3_EVIDENCE",e.sequence);
                return;
            }
            if (m=="PX4: GPS 3D fixed & EKF initialized.")
            {
                readyCount++;e.interpretation="VENDOR_EKF_READY_REPORT_NOT_FLIGHT_ADMISSION";
                if(startLine<=boundary) {Reject("READY_WITHOUT_CURRENT_SIM_START",e.sequence);return;}
                if(wall>=60.0) {Reject("READY_OBSERVED_AFTER_INITIALIZATION_DEADLINE",e.sequence);return;}
                readyLine=e.sequence;readyTime=time;return;
            }
            // Obtain preflight and sensor health from MAVLink and disarmed observation.
            if (m.IndexOf("Preflight Fail:",StringComparison.OrdinalIgnoreCase)>=0 ||
                Regex.IsMatch(m,@"\b(?:MAG|BARO|GYRO|ACCEL)\s*#\d+\s+failed:",RegexOptions.IgnoreCase))
            {e.interpretation="INDEPENDENT_HEALTH_DIAGNOSTIC_ONLY";notes.Add(e);return;}
            // Optional feature text such as DllFaultParamAPI false is not a crash.
            // Required APIs are checked above.
            if (Regex.IsMatch(m,@"^(?:Dll\w+|is\w+)\s+(?:true|false)!$"))
            {e.interpretation="OPTIONAL_VENDOR_API_METADATA";return;}
            if (Regex.IsMatch(m,@"\b(?:fatal|crash|hard\s*fault|segmentation fault|access violation|unhandled exception)\b",RegexOptions.IgnoreCase) ||
                e.severity.Equals("fatal",StringComparison.OrdinalIgnoreCase) || e.severity.Equals("critical",StringComparison.OrdinalIgnoreCase))
            {e.interpretation="EXPLICIT_FATAL_OR_CRASH";Reject(e.interpretation,e.sequence);return;}
            if (Regex.IsMatch(m,@"(?:serial|COM\d|SerialPort).*(?:error|failed|failure|denied|exception|cannot|unable|not successful)",RegexOptions.IgnoreCase) ||
                Regex.IsMatch(m,@"(?:error|failed|failure|denied|exception|cannot|unable).*(?:serial|COM\d|SerialPort)",RegexOptions.IgnoreCase))
            {e.interpretation="EXPLICIT_SERIAL_OR_COM_ERROR";Reject(e.interpretation,e.sequence);return;}
            if (m.StartsWith("PX4: GPS 3D fixed & EKF",StringComparison.Ordinal) ||
                m.StartsWith("Send reboot",StringComparison.Ordinal) || m.StartsWith("model Stop",StringComparison.Ordinal))
            {e.interpretation="UNKNOWN_VENDOR_LIFECYCLE_STATE";Reject(e.interpretation,e.sequence);return;}
            e.interpretation="NONADMISSION_TEXT_RETAINED";unknown++;
        }
    }
}
