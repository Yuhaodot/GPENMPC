#include <windows.h>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <stdexcept>

// Probe the generated CopterSim DLL ABI and state machine without sockets or serial.
template<class T> T sym(HMODULE dll, const char* name) {
    FARPROC p = GetProcAddress(dll, name);
    if (!p) throw std::runtime_error(name);
    return reinterpret_cast<T>(p);
}

struct Api {
    void (*init)();
    void (*init_pos)(const double*, const double*);
    void (*init_gps)(const double*);
    void (*pwm)(const double*);
    void (*terrain)(double*);
    void (*environment)(const double*);
    void (*step)();
    void (*output)(double*);
    double (*step_size)();
    void (*destroy)();
};

static void start(Api& a) {
    const double xyz[3] = {0.0, 0.0, 0.0};
    const double rpy[3] = {0.0, 0.0, 0.0};
    const double gps[3] = {40.1540302, 116.2593683, 50.0};
    const double motor[16] = {};
    const double empty_environment[28] = {};
    double ground[15] = {};
    // Clear the separate 28-D external-input storage before and after initialize
    // so each fixture starts independently of the previous one.
    a.environment(empty_environment);
    a.init_pos(xyz, rpy);
    a.init_gps(gps);
    a.init();
    a.environment(empty_environment);
    a.pwm(motor);
    a.terrain(ground);
    if (std::abs(a.step_size() - 0.001) > 1e-12) throw std::runtime_error("unexpected_model_step");
}

static void frame(double f[28], std::uint32_t generation, double source_time,
                  double payload, std::uint32_t payload_generation,
                  std::uint32_t release_generation, double session = 26090501.0) {
    for (int i = 0; i < 28; ++i) f[i] = 0.0;
    f[0] = 2.0;
    f[1] = static_cast<double>(generation);
    f[2] = source_time;
    f[3] = 0.0;               // paused task clock
    f[4] = payload;
    f[7] = 2.0;               // service/ground phase
    f[20] = static_cast<double>(payload_generation);
    f[21] = 1.0;              // task clock paused
    f[22] = session;
    f[23] = source_time;       // fresh min(HB, EXT) receive time
    f[24] = 15.0;              // Heartbeat, landed state, identity and clock valid; disarmed.
    f[25] = 1.0;               // PX4 ON_GROUND
    f[26] = 1.0;               // uninterrupted continuity epoch
    f[27] = static_cast<double>(release_generation);
}

static bool finite32(const double x[32]) {
    for (int i = 0; i < 32; ++i) if (!std::isfinite(x[i])) return false;
    return true;
}

int wmain(int argc, wchar_t** argv) {
    if (argc != 4) return 2;
    HMODULE dll = LoadLibraryW(argv[1]);
    if (!dll) { std::printf("LoadLibraryW error=%lu\n", GetLastError()); return 3; }
    FILE* csv = _wfopen(argv[2], L"wb");
    FILE* json = _wfopen(argv[3], L"wb");
    if (!csv || !json) { if (csv) std::fclose(csv); if (json) std::fclose(json); FreeLibrary(dll); return 4; }
    int rc = 0;
    int checks = 0, passed = 0;
    auto check = [&](bool ok, const char* name) {
        ++checks;
        if (ok) { ++passed; std::printf("PASS %s\n", name); }
        else { std::printf("FAIL %s\n", name); rc = 5; }
    };
    try {
        auto bind = [&]() -> Api { return Api{
                sym<void(*)()>(dll, "DllReInitModel"),
                sym<void(*)(const double*,const double*)>(dll, "DllInitPosAngState"),
                sym<void(*)(const double*)>(dll, "DllInitGpsPos"),
                sym<void(*)(const double*)>(dll, "DllInputPWMs"),
                sym<void(*)(double*)>(dll, "DllTerrainIn15d"),
                sym<void(*)(const double*)>(dll, "DllInputDoubCtrls"),
                sym<void(*)()>(dll, "Dllstep"),
                sym<void(*)(double*)>(dll, "DllOutCopterData"),
                sym<double(*)()>(dll, "DllGetStep0"),
                sym<void(*)()>(dll, "DllDestroyModel")
            }; };
        auto reload = [&]() {
            if (dll) FreeLibrary(dll);
            dll = LoadLibraryW(argv[1]);
            if (!dll) throw std::runtime_error("fixture_dll_reload");
        };
        Api a = bind();
        wchar_t selected[32] = {};
        GetEnvironmentVariableW(L"GPENMPC_PROBE_CASE", selected, 32);
        const bool run_normal = selected[0] == 0 || _wcsicmp(selected, L"NORMAL") == 0;
        const bool run_premature = selected[0] == 0 || _wcsicmp(selected, L"PREMATURE") == 0;
        const bool run_wrong_session = selected[0] == 0 || _wcsicmp(selected, L"WRONG_SESSION") == 0;
        double out[32] = {}, good[28] = {};
        std::fprintf(csv, "case,step,time_s,session,frame_generation,payload_generation,payload_kg,total_mass_kg,status\n");

        // Case 1: one continuous plant, eight seconds grounded, then one unload.
        if (run_normal) {
        start(a);
        double z[28] = {};
        a.environment(z); a.step(); a.output(out);
        check(finite32(out), "initial_output_finite");
        check(out[25] == 2.0 && out[31] == 16.0 && out[26] == 0.0,
              "unbound_delivery_extension_visible");
        for (int k = 0; k < 8300; ++k) {
            const double t = (k + 1) * 0.001;
            const std::uint32_t generation = 1u + static_cast<std::uint32_t>(k / 10);
            const bool released = k >= 8099;
            double f[28];
            frame(f, generation, t, released ? 1.75 : 2.21, released ? 1u : 0u, released ? 1u : 0u);
            a.environment(f); a.step(); a.output(out);
            if ((k % 10) == 9 || k == 8099 || k == 8299)
                std::fprintf(csv, "NORMAL,%d,%.6f,%.0f,%.0f,%.0f,%.12g,%.12g,%.0f\n",
                             k + 1, t, out[26], out[27], out[28], out[29], out[30], out[31]);
        }
        check(finite32(out), "normal_final_output_finite");
        check(out[25] == 2.0 && out[26] == 26090501.0 && out[28] == 1.0,
              "normal_session_and_payload_generation_bound");
        check(std::abs(out[29] - 1.75) < 1e-12 && std::abs(out[30] - 11.25) < 1e-12,
              "actual_mass_changed_without_reset");
        check(out[31] == 0.0, "normal_delivery_status_healthy");
        const double normal_time = out[2];
        check(normal_time > 8.2, "same_model_clock_continued_across_unload");
        a.destroy();
        }

        // Case 2: premature release is latched fail-closed and cannot re-BEGIN.
        if (run_premature) {
        if (run_normal) { reload(); a = bind(); }
        start(a);
        double empty[28] = {};
        a.environment(empty); a.step(); a.output(out); // physical-reset tick
        for (int k = 0; k < 1200; ++k) {
            const double t = (k + 2) * 0.001;
            const std::uint32_t generation = 1u + static_cast<std::uint32_t>(k / 10);
            const bool released = k >= 999;
            double f[28]; frame(f, generation, t, released ? 1.75 : 2.21, released ? 1u : 0u, released ? 1u : 0u);
            a.environment(f); a.step(); a.output(out);
        }
        std::printf("OBS premature session=%.0f frame=%.0f pgen=%.0f payload=%.12g mass=%.12g status=%.0f time=%.6f\n",
                    out[26], out[27], out[28], out[29], out[30], out[31], out[2]);
        check(out[31] == 10.0 && out[28] == 0.0 && std::abs(out[29] - 2.21) < 1e-12,
              "premature_unload_rejected_without_mass_change");
        frame(good, 1u, 1.201, 2.21, 0u, 0u);
        a.environment(good); a.step(); a.output(out);
        check(out[31] == 10.0 && out[26] == 26090501.0,
              "premature_failure_cannot_be_rebegun");
        a.destroy();
        }

        // Case 3: wrong session is a permanent identity fault for that instance.
        if (run_wrong_session) {
        if (run_normal || run_premature) { reload(); a = bind(); }
        start(a);
        double empty[28] = {};
        a.environment(empty); a.step(); a.output(out); // physical-reset tick
        double bad[28]; frame(bad, 1u, 0.002, 2.21, 0u, 0u, 26090502.0);
        a.environment(bad);
        for (int k = 0; k < 20; ++k) { a.step(); a.output(out); }
        std::printf("OBS wrong_session session=%.0f frame=%.0f pgen=%.0f payload=%.12g mass=%.12g status=%.0f time=%.6f\n",
                    out[26], out[27], out[28], out[29], out[30], out[31], out[2]);
        check(out[31] == 12.0 && out[27] == 0.0 && out[28] == 0.0,
              "wrong_session_rejected_before_binding");
        frame(good, 1u, 0.022, 2.21, 0u, 0u);
        a.environment(good);
        for (int k = 0; k < 20; ++k) { a.step(); a.output(out); }
        check(out[31] == 12.0 && out[27] == 0.0 && out[28] == 0.0,
              "wrong_session_cannot_be_washed_by_good_frame");
        a.destroy();
        }
    } catch (const std::exception& e) {
        std::printf("FAIL_HOST_DLL_PROBE %s\n", e.what());
        rc = 6;
    }
    const bool ok = rc == 0 && checks == passed;
    std::fprintf(json,
        "{\n  \"schema\": \"HOST_CANONICAL_DELIVERY_DLL_DYNAMIC_PROBE_V1\",\n"
        "  \"status\": \"%s\",\n  \"pass\": %s,\n  \"checks_total\": %d,\n  \"checks_passed\": %d,\n"
        "  \"dll_input_doub_ctrls_called\": true,\n  \"normal_steps\": 8300,\n"
        "  \"normal_unload_dwell_s\": 8.0,\n  \"expected_post_unload_payload_kg\": 1.75,\n"
        "  \"expected_post_unload_total_mass_kg\": 11.25,\n"
        "  \"COM_open\": 0,\n  \"UDP_open\": 0,\n  \"board_actions\": 0,\n"
        "  \"limitations\": \"Direct generated-DLL HOST probe.\"\n}\n",
        ok ? "PASS_HOST_GENERATED_DLL_DELIVERY_STATE" : "FAIL_HOST_GENERATED_DLL_DELIVERY_STATE",
        ok ? "true" : "false", checks, passed);
    std::fclose(csv); std::fclose(json); FreeLibrary(dll);
    std::printf("%s checks=%d/%d\n", ok ? "PASS_HOST_DLL_DELIVERY" : "FAIL_HOST_DLL_DELIVERY", passed, checks);
    return ok ? 0 : (rc ? rc : 7);
}
