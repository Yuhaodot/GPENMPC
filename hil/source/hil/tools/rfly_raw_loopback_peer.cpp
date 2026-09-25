// Test RflyUdpRaw with a fixed-port loopback peer.
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

struct Packet {
    std::uint32_t length{};
    std::int64_t qpc{};
    unsigned char bytes[300]{};
};

static bool exists(const std::wstring& p)
{
    return GetFileAttributesW(p.c_str()) != INVALID_FILE_ATTRIBUTES;
}

static bool text_file(const std::wstring& p, const char* text)
{
    if (exists(p)) return false;
    FILE* f = _wfopen(p.c_str(), L"wb");
    if (!f) return false;
    const auto n = std::strlen(text);
    const bool wrote = std::fwrite(text, 1, n, f) == n;
    const bool closed = std::fclose(f) == 0;
    return wrote && closed;
}

int wmain(int argc, wchar_t** argv)
{
    if (argc != 3) return 2;
    const std::wstring input_path(argv[1]), directory(argv[2]);
    const auto attr = GetFileAttributesW(directory.c_str());
    if (attr == INVALID_FILE_ATTRIBUTES || !(attr & FILE_ATTRIBUTE_DIRECTORY)) return 2;
    const std::wstring ready_path = directory + L"\\READY.json";
    const std::wstring stop_path = directory + L"\\STOP.txt";
    const std::wstring raw_path = directory + L"\\PEER_RAW.bin";
    const std::wstring result_path = directory + L"\\PEER_RESULT.json";
    if (exists(ready_path) || exists(stop_path) || exists(raw_path) || exists(result_path)) return 2;
    unsigned char expected[301]{};
    FILE* input = _wfopen(input_path.c_str(), L"rb");
    if (!input) return 2;
    const auto expected_size = std::fread(expected, 1, sizeof(expected), input);
    const bool input_ok = std::ferror(input) == 0;
    std::fclose(input);
    if (!input_ok || expected_size < 6 || expected_size > 300) return 2;
    std::vector<Packet> packets;
    packets.reserve(8000); // bounded fixture memory, no per-packet filesystem IO
    WSADATA wsa{};
    int error_code = WSAStartup(MAKEWORD(2, 2), &wsa);
    const bool wsa_started = error_code == 0;
    SOCKET sock = INVALID_SOCKET;
    bool bind_ok = false, close_ok = false, stopped = false, timeout = false;
    std::uint32_t received = 0, echoed = 0, unexpected = 0;
    LARGE_INTEGER frequency{}, start{}, now{};
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);
    if (!error_code) {
        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock == INVALID_SOCKET) error_code = WSAGetLastError();
    }
    if (!error_code) {
        BOOL exclusive = TRUE;
        if (setsockopt(sock, SOL_SOCKET, SO_EXCLUSIVEADDRUSE,
                       reinterpret_cast<const char*>(&exclusive), sizeof(exclusive)) != 0)
            error_code = WSAGetLastError();
    }
    sockaddr_in local{}, destination{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    local.sin_port = htons(62321);
    destination.sin_family = AF_INET;
    destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    destination.sin_port = htons(62322);
    if (!error_code) {
        bind_ok = bind(sock, reinterpret_cast<sockaddr*>(&local), sizeof(local)) == 0;
        if (!bind_ok) error_code = WSAGetLastError();
    }
    if (!error_code) {
        u_long nonblocking = 1;
        if (ioctlsocket(sock, FIONBIO, &nonblocking) != 0) error_code = WSAGetLastError();
    }
    if (!error_code && !text_file(ready_path, "{\"ready\":true,\"loopback_only\":true}\n"))
        error_code = ERROR_WRITE_FAULT;
    while (!error_code) {
        QueryPerformanceCounter(&now);
        if (static_cast<double>(now.QuadPart - start.QuadPart) / frequency.QuadPart >= 30.0) {
            timeout = true;
            break;
        }
        if (exists(stop_path)) { stopped = true; break; }
        unsigned char bytes[301]{};
        sockaddr_in sender{};
        int sender_length = sizeof(sender);
        const int n = recvfrom(sock, reinterpret_cast<char*>(bytes), sizeof(bytes), 0,
                               reinterpret_cast<sockaddr*>(&sender), &sender_length);
        if (n == SOCKET_ERROR) {
            const int error = WSAGetLastError();
            if (error == WSAEWOULDBLOCK) { Sleep(1); continue; }
            error_code = error;
            break;
        }
        ++received;
        if (sender.sin_family != AF_INET || sender.sin_addr.s_addr != htonl(INADDR_LOOPBACK) ||
            sender.sin_port != htons(62322) || n != static_cast<int>(expected_size) ||
            std::memcmp(bytes, expected, expected_size) != 0 || packets.size() >= 8000) {
            ++unexpected;
            error_code = ERROR_INVALID_DATA;
            break;
        }
        Packet packet{};
        packet.length = static_cast<std::uint32_t>(n);
        QueryPerformanceCounter(&now);
        packet.qpc = now.QuadPart;
        std::memcpy(packet.bytes, bytes, static_cast<std::size_t>(n));
        packets.push_back(packet);
        const int sent = sendto(sock, reinterpret_cast<const char*>(packet.bytes), n, 0,
                                 reinterpret_cast<sockaddr*>(&destination), sizeof(destination));
        if (sent != n) {
            error_code = sent == SOCKET_ERROR ? WSAGetLastError() : ERROR_WRITE_FAULT;
            break;
        }
        ++echoed;
    }
    if (sock != INVALID_SOCKET) {
        close_ok = closesocket(sock) == 0;
        if (!close_ok && !error_code) error_code = WSAGetLastError();
    }
    if (wsa_started && WSACleanup() != 0 && !error_code) error_code = WSAGetLastError();
    QueryPerformanceCounter(&now);
    bool raw_ok = false;
    if (!exists(raw_path)) {
        FILE* raw = _wfopen(raw_path.c_str(), L"wb");
        if (raw) {
            raw_ok = true;
            for (const auto& p : packets) {
                raw_ok = raw_ok && std::fwrite(&p.length, sizeof(p.length), 1, raw) == 1;
                raw_ok = raw_ok && std::fwrite(&p.qpc, sizeof(p.qpc), 1, raw) == 1;
                raw_ok = raw_ok && std::fwrite(p.bytes, 1, p.length, raw) == p.length;
            }
            raw_ok = std::fclose(raw) == 0 && raw_ok;
        }
    }
    if (!raw_ok && !error_code) error_code = ERROR_WRITE_FAULT;
    char result[1600]{};
    std::snprintf(result, sizeof(result),
        "{\"scope\":\"HOST_LOOPBACK_EXACT_BYTE_ECHO_FIXTURE\","
        "\"received_count\":%u,\"echoed_count\":%u,\"unexpected_count\":%u,"
        "\"bind_ok\":%s,\"close_ok\":%s,\"error_code\":%d,"
        "\"stop_requested\":%s,\"hard_timeout\":%s,\"hard_wall_bound_s\":30,"
        "\"qpc_frequency\":%lld,\"elapsed_s\":%.9f,\"expected_packet_bytes\":%zu,"
        "\"record_format\":\"uint32LE_length_int64LE_QPC_ticks_raw_bytes\","
        "\"raw_written\":%s,\"COM\":0,\"board\":0,\"plant\":0}\n",
        received, echoed, unexpected, bind_ok ? "true" : "false", close_ok ? "true" : "false",
        error_code, stopped ? "true" : "false", timeout ? "true" : "false",
        static_cast<long long>(frequency.QuadPart),
        static_cast<double>(now.QuadPart - start.QuadPart) / frequency.QuadPart,
        expected_size, raw_ok ? "true" : "false");
    const bool result_ok = text_file(result_path, result);
    return error_code == 0 && stopped && !timeout && close_ok && result_ok ? 0 : 1;
}
