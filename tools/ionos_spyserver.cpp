// SPDX-License-Identifier: MIT
//
// ionos_spyserver.cpp - SpyServer bridge for a SoapySDR device (default driver=ionos)
//   Copyright (c) 2026 Zoltan Doczi HA7DCD
//
// Serves the device's RX stream over the SpyServer protocol, so any SpyServer
// client (SDR++'s built-in spyserver_source, SDR#, SDRangel, ...) can use an
// Ionos SDR on USB without a client-side module:
//
//   ionos_spyserver                         (driver=ionos, 127.0.0.1:5555)
//   ionos_spyserver --bind 0.0.0.0 --port 5555 --args driver=ionos,port=COM9
//
// Protocol notes (as SDR++'s client uses it):
//   - DeviceInfo.MaximumSampleRate >> i, i = MinimumIQDecimation..DecimationStageCount,
//     is the client's rate list. Every such rate is a native device rate here
//     (no software decimation), so the stage count is the number of halvings
//     of the top rate the device offers.
//   - The IQ format the client asks for (UINT8 / INT16 / FLOAT) is sent; the
//     message type tells the client which one it got.
//   - GainStageCount = MaximumGainIndex = 0: the FG23 runs its own AGC, the
//     client then shows no gain slider. IQ_DIGITAL_GAIN is accepted and
//     ignored (the message flags stay 0, so the client scale is 1).
//   - One client at a time; a second client waits until the first disconnects.
//   - The sender never blocks the device reader: whole messages are queued
//     (about 1 s) and dropped whole when the client falls behind. A message is
//     never cut, the client could not resynchronise.

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Version.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET sock_t;
#define SOCK_INVALID INVALID_SOCKET
#define sock_close closesocket
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
typedef int sock_t;
#define SOCK_INVALID (-1)
#define sock_close close
#endif

namespace {

// ---------------------------------------------------------------- protocol
const uint32_t PROTOCOL_VERSION = (2u << 24) | (0u << 16) | 1700u;

enum : uint32_t { CMD_HELLO = 0, CMD_GET_SETTING = 1, CMD_SET_SETTING = 2, CMD_PING = 3 };
enum : uint32_t {
    MSG_DEVICE_INFO = 0, MSG_CLIENT_SYNC = 1, MSG_PONG = 2,
    MSG_UINT8_IQ = 100, MSG_INT16_IQ = 101, MSG_FLOAT_IQ = 103,
};
enum : uint32_t { STREAM_STATUS = 0, STREAM_IQ = 1 };
enum : uint32_t { FMT_UINT8 = 1, FMT_INT16 = 2, FMT_FLOAT = 4 };
enum : uint32_t {
    SET_STREAMING_MODE = 0, SET_STREAMING_ENABLED = 1, SET_GAIN = 2,
    SET_IQ_FORMAT = 100, SET_IQ_FREQUENCY = 101, SET_IQ_DECIMATION = 102, SET_IQ_DIGITAL_GAIN = 103,
};
const uint32_t DEVICE_RTLSDR = 3;  // a type every client knows; see the header note

#pragma pack(push, 1)
struct MsgHeader { uint32_t ProtocolID, MessageType, StreamType, SequenceNumber, BodySize; };
struct DeviceInfo {
    uint32_t DeviceType, DeviceSerial, MaximumSampleRate, MaximumBandwidth,
        DecimationStageCount, GainStageCount, MaximumGainIndex, MinimumFrequency,
        MaximumFrequency, Resolution, MinimumIQDecimation, ForcedIQFormat;
};
struct ClientSync {
    uint32_t CanControl, Gain, DeviceCenterFrequency, IQCenterFrequency, FFTCenterFrequency,
        MinimumIQCenterFrequency, MaximumIQCenterFrequency, MinimumFFTCenterFrequency,
        MaximumFFTCenterFrequency;
};
#pragma pack(pop)

// ---------------------------------------------------------------- options
struct Opts {
    std::string args = "driver=ionos";
    std::string bind = "127.0.0.1";
    uint16_t port = 5555;
    double queueSecs = 1.0;
    bool verbose = false;
};

std::atomic<bool> g_quit{false};

void onSignal(int) { g_quit = true; }

void logf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    std::vfprintf(stderr, fmt, ap);
    va_end(ap);
    std::fputc('\n', stderr);
    std::fflush(stderr);
}

uint32_t fnv1a(const std::string &s) {
    uint32_t h = 2166136261u;
    for (unsigned char c : s) { h ^= c; h *= 16777619u; }
    return h;
}

// ---------------------------------------------------------------- device
struct Device {
    SoapySDR::Device *dev = nullptr;
    SoapySDR::Stream *stream = nullptr;
    std::vector<double> rates;  // rate for decimation stage i (index = stage)
    uint32_t fmin = 0, fmax = 0, serial = 0;
    std::string label;

    bool open(const std::string &args) {
        try {
            dev = SoapySDR::Device::make(args);
        } catch (const std::exception &e) {
            logf("ionos_spyserver: cannot open \"%s\": %s", args.c_str(), e.what());
            return false;
        }
        if (!dev) return false;

        std::vector<double> list = dev->listSampleRates(SOAPY_SDR_RX, 0);
        if (list.empty()) {
            for (const auto &r : dev->getSampleRateRange(SOAPY_SDR_RX, 0)) list.push_back(r.maximum());
        }
        if (list.empty()) { logf("ionos_spyserver: device lists no sample rates"); return false; }
        const double top = *std::max_element(list.begin(), list.end());
        rates.push_back(top);
        for (int i = 1; i < 16; i++) {
            const double want = top / double(1u << i);
            auto it = std::find_if(list.begin(), list.end(), [&](double r) { return std::fabs(r - want) < 1.0; });
            if (it == list.end()) break;
            rates.push_back(*it);
        }

        double lo = 1e18, hi = 0;
        for (const auto &r : dev->getFrequencyRange(SOAPY_SDR_RX, 0)) {
            lo = std::min(lo, r.minimum());
            hi = std::max(hi, r.maximum());
        }
        if (hi <= 0) { lo = 0; hi = 4e9; }
        fmin = uint32_t(std::max(0.0, lo));
        fmax = uint32_t(std::min(4294967295.0, hi));

        const auto info = dev->getHardwareInfo();
        std::string ser = dev->getHardwareKey();
        auto it = info.find("serial");
        if (it != info.end()) ser += it->second;
        auto pit = info.find("port");
        if (pit != info.end()) ser += pit->second;
        serial = fnv1a(ser);
        label = dev->getHardwareKey();
        auto bit = info.find("board");
        if (bit != info.end()) label += " [" + bit->second + "]";

        // The FG23 runs its own AGC; leave a hardware AGC on where there is one.
        try { if (dev->hasGainMode(SOAPY_SDR_RX, 0)) dev->setGainMode(SOAPY_SDR_RX, 0, true); } catch (...) {}
        return true;
    }

    void close() {
        if (stream) { try { dev->deactivateStream(stream); dev->closeStream(stream); } catch (...) {} stream = nullptr; }
        if (dev) { SoapySDR::Device::unmake(dev); dev = nullptr; }
    }
};

// ---------------------------------------------------------------- session
class Session {
public:
    Session(Device &d, sock_t s, const Opts &o) : dev(d), sock(s), opts(o) {}

    void run() {
        freq = uint32_t(std::clamp(dev.dev->getFrequency(SOAPY_SDR_RX, 0), double(dev.fmin), double(dev.fmax)));
        std::thread sender(&Session::sendLoop, this);
        std::thread streamer(&Session::streamLoop, this);
        recvLoop();
        alive = false;
        qcv.notify_all();
        streamer.join();
        sender.join();
        if (dropped) logf("ionos_spyserver: %llu message(s) dropped for this client (slow reader)",
                          (unsigned long long)dropped.load());
    }

private:
    Device &dev;
    sock_t sock;
    const Opts &opts;
    std::atomic<bool> alive{true};

    // client state (written by recvLoop, read by streamLoop)
    std::mutex stMtx;
    bool streaming = false, iqMode = true, dirty = true;
    uint32_t fmt = FMT_INT16, stage = 0, freq = 0;

    // output queue
    std::mutex qMtx;
    std::condition_variable qcv;
    std::deque<std::vector<uint8_t>> q;
    size_t qBytes = 0;
    uint32_t seq = 0;
    std::atomic<uint64_t> dropped{0};

    size_t queueLimit() const {
        const double r = dev.rates.empty() ? 200000.0 : dev.rates[0];
        return size_t(std::max(256.0 * 1024, r * 8 * opts.queueSecs));  // 8 B/sample covers FLOAT
    }

    void push(uint32_t type, uint32_t stream, const void *body, uint32_t len, bool mayDrop) {
        std::vector<uint8_t> m(sizeof(MsgHeader) + len);
        std::lock_guard<std::mutex> lk(qMtx);
        if (mayDrop && qBytes + m.size() > queueLimit()) { dropped++; return; }
        MsgHeader h{PROTOCOL_VERSION, type, stream, seq++, len};
        std::memcpy(m.data(), &h, sizeof h);
        if (len) std::memcpy(m.data() + sizeof h, body, len);
        qBytes += m.size();
        q.push_back(std::move(m));
        qcv.notify_one();
    }

    void sendDeviceInfo() {
        DeviceInfo d{};
        d.DeviceType = DEVICE_RTLSDR;
        d.DeviceSerial = dev.serial;
        d.MaximumSampleRate = uint32_t(dev.rates[0]);
        d.MaximumBandwidth = uint32_t(dev.rates[0]);
        d.DecimationStageCount = uint32_t(dev.rates.size() - 1);
        d.GainStageCount = 0;
        d.MaximumGainIndex = 0;
        d.MinimumFrequency = dev.fmin;
        d.MaximumFrequency = dev.fmax;
        d.Resolution = 16;
        d.MinimumIQDecimation = 0;
        d.ForcedIQFormat = 0;
        push(MSG_DEVICE_INFO, STREAM_STATUS, &d, sizeof d, false);
    }

    void sendClientSync(uint32_t f) {
        ClientSync c{};
        c.CanControl = 1;
        c.DeviceCenterFrequency = c.IQCenterFrequency = c.FFTCenterFrequency = f;
        c.MinimumIQCenterFrequency = c.MinimumFFTCenterFrequency = dev.fmin;
        c.MaximumIQCenterFrequency = c.MaximumFFTCenterFrequency = dev.fmax;
        push(MSG_CLIENT_SYNC, STREAM_STATUS, &c, sizeof c, false);
    }

    bool recvAll(void *buf, size_t n) {
        auto *p = static_cast<char *>(buf);
        while (n > 0) {
            const int r = ::recv(sock, p, int(n), 0);
            if (r <= 0) return false;
            p += r;
            n -= size_t(r);
        }
        return true;
    }

    void recvLoop() {
        while (alive && !g_quit) {
            uint32_t hdr[2];
            if (!recvAll(hdr, sizeof hdr)) return;
            const uint32_t type = hdr[0], len = hdr[1];
            if (len > 4096) { logf("ionos_spyserver: command body too large (%u B), closing", len); return; }
            std::vector<uint8_t> body(len);
            if (len && !recvAll(body.data(), len)) return;
            handleCommand(type, body);
        }
    }

    void handleCommand(uint32_t type, const std::vector<uint8_t> &b) {
        switch (type) {
        case CMD_HELLO: {
            uint32_t ver = 0;
            if (b.size() >= 4) std::memcpy(&ver, b.data(), 4);
            const std::string name(b.begin() + std::min<size_t>(4, b.size()), b.end());
            logf("ionos_spyserver: HELLO from \"%s\" (protocol %u.%u.%u)", name.c_str(), ver >> 24,
                 (ver >> 16) & 0xFF, ver & 0xFFFF);
            uint32_t f;
            {
                std::lock_guard<std::mutex> lk(stMtx);
                streaming = false;
                stage = 0;
                dirty = true;
                f = freq;
            }
            sendDeviceInfo();
            sendClientSync(f);
            break;
        }
        case CMD_SET_SETTING: {
            if (b.size() < 8) break;
            uint32_t id, v;
            std::memcpy(&id, b.data(), 4);
            std::memcpy(&v, b.data() + 4, 4);
            handleSetting(id, v);
            break;
        }
        case CMD_GET_SETTING: {
            uint32_t f;
            { std::lock_guard<std::mutex> lk(stMtx); f = freq; }
            sendClientSync(f);
            break;
        }
        case CMD_PING:
            push(MSG_PONG, STREAM_STATUS, nullptr, 0, false);
            break;
        default:
            if (opts.verbose) logf("ionos_spyserver: unknown command %u", type);
        }
    }

    void handleSetting(uint32_t id, uint32_t v) {
        bool sync = false;
        uint32_t f = 0;
        {
            std::lock_guard<std::mutex> lk(stMtx);
            switch (id) {
            case SET_STREAMING_MODE: iqMode = (v & STREAM_IQ) != 0; dirty = true; break;
            case SET_STREAMING_ENABLED: streaming = v != 0; dirty = true; break;
            case SET_IQ_FORMAT:
                if (v == FMT_UINT8 || v == FMT_INT16 || v == FMT_FLOAT) fmt = v;
                else logf("ionos_spyserver: IQ format %u not supported, sending INT16", v), fmt = FMT_INT16;
                break;
            case SET_IQ_FREQUENCY:
                freq = std::clamp(v, dev.fmin, dev.fmax);
                dirty = true;
                sync = true;
                break;
            case SET_IQ_DECIMATION:
                stage = std::min<uint32_t>(v, uint32_t(dev.rates.size() - 1));
                dirty = true;
                sync = true;
                break;
            case SET_GAIN:
            case SET_IQ_DIGITAL_GAIN:
                break;  // AGC on the device; see the header note
            default:
                if (opts.verbose) logf("ionos_spyserver: setting %u = %u ignored", id, v);
            }
            f = freq;
        }
        if (opts.verbose) logf("ionos_spyserver: setting %u = %u", id, v);
        if (sync) sendClientSync(f);
    }

    void streamLoop() {
        std::vector<int16_t> iq;
        std::vector<uint8_t> out;
        bool active = false;
        uint32_t curStage = UINT32_MAX, curFreq = 0, curFmt = FMT_INT16;
        bool want = false;
        size_t perMsg = 2048;
        auto lastLog = std::chrono::steady_clock::now();
        uint64_t sent = 0;

        while (alive && !g_quit) {
            {
                std::lock_guard<std::mutex> lk(stMtx);
                if (dirty) {
                    dirty = false;
                    want = streaming && iqMode;
                    try {
                        if (freq != curFreq) {
                            dev.dev->setFrequency(SOAPY_SDR_RX, 0, double(freq));
                            curFreq = freq;
                            if (opts.verbose) logf("ionos_spyserver: tuned %u Hz", freq);
                        }
                        if (stage != curStage) {
                            if (active) { dev.dev->deactivateStream(dev.stream); active = false; }
                            dev.dev->setSampleRate(SOAPY_SDR_RX, 0, dev.rates[stage]);
                            curStage = stage;
                            const double r = dev.rates[stage];
                            perMsg = size_t(std::clamp(r / 100.0, 256.0, 16384.0));  // ~10 ms
                            logf("ionos_spyserver: %.0f sps (decimation stage %u)", r, stage);
                        }
                    } catch (const std::exception &e) {
                        logf("ionos_spyserver: device setting failed: %s", e.what());
                    }
                }
                curFmt = fmt;
            }

            if (want && !active) {
                if (!dev.stream) dev.stream = dev.dev->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16);
                dev.dev->activateStream(dev.stream);
                active = true;
            } else if (!want && active) {
                dev.dev->deactivateStream(dev.stream);
                active = false;
            }
            if (!active) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            iq.resize(perMsg * 2);
            size_t got = 0;
            while (got < perMsg && alive) {
                void *bufs[] = {iq.data() + 2 * got};
                int flags = 0;
                long long t = 0;
                const int r = dev.dev->readStream(dev.stream, bufs, perMsg - got, flags, t, 200000);
                if (r > 0) got += size_t(r);
                else if (r == SOAPY_SDR_TIMEOUT || r == SOAPY_SDR_OVERFLOW) { if (r == SOAPY_SDR_TIMEOUT) break; }
                else { logf("ionos_spyserver: readStream error %d", r); std::this_thread::sleep_for(std::chrono::milliseconds(50)); break; }
            }
            if (got == 0) continue;

            uint32_t type;
            switch (curFmt) {
            case FMT_UINT8:
                type = MSG_UINT8_IQ;
                out.resize(got * 2);
                for (size_t i = 0; i < got * 2; i++) out[i] = uint8_t((iq[i] >> 8) + 128);
                break;
            case FMT_FLOAT: {
                type = MSG_FLOAT_IQ;
                out.resize(got * 2 * sizeof(float));
                auto *f = reinterpret_cast<float *>(out.data());
                for (size_t i = 0; i < got * 2; i++) f[i] = float(iq[i]) * (1.0f / 32768.0f);
                break;
            }
            default:
                type = MSG_INT16_IQ;
                out.resize(got * 2 * sizeof(int16_t));
                std::memcpy(out.data(), iq.data(), out.size());
            }
            push(type, STREAM_IQ, out.data(), uint32_t(out.size()), true);
            sent += got;

            if (opts.verbose && std::chrono::steady_clock::now() - lastLog > std::chrono::seconds(5)) {
                lastLog = std::chrono::steady_clock::now();
                logf("ionos_spyserver: %llu samples queued, %llu dropped msgs", (unsigned long long)sent,
                     (unsigned long long)dropped.load());
            }
        }
        if (active) { try { dev.dev->deactivateStream(dev.stream); } catch (...) {} }
    }

    void sendLoop() {
        while (true) {
            std::vector<uint8_t> m;
            {
                std::unique_lock<std::mutex> lk(qMtx);
                qcv.wait_for(lk, std::chrono::milliseconds(200), [&] { return !q.empty() || !alive; });
                if (!alive && q.empty()) return;
                if (q.empty()) continue;
                m = std::move(q.front());
                q.pop_front();
                qBytes -= m.size();
            }
            const char *p = reinterpret_cast<const char *>(m.data());
            size_t n = m.size();
            while (n > 0) {
                const int w = ::send(sock, p, int(n), 0);
                if (w <= 0) { alive = false; return; }
                p += w;
                n -= size_t(w);
            }
        }
    }
};

void usage() {
    std::fprintf(stderr,
                 "ionos_spyserver - SpyServer bridge for a SoapySDR device\n"
                 "  --args <soapy args>   device (default driver=ionos)\n"
                 "  --bind <addr>         listen address (default 127.0.0.1; 0.0.0.0 = all)\n"
                 "  --port <n>            TCP port (default 5555)\n"
                 "  --queue <s>           send queue length in seconds (default 1)\n"
                 "  -v                    verbose\n");
}

}  // namespace

int main(int argc, char **argv) {
    Opts o;
    for (int i = 1; i < argc; i++) {
        const std::string k = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) { usage(); std::exit(2); }
            return argv[++i];
        };
        if (k == "--args") o.args = next();
        else if (k == "--bind") o.bind = next();
        else if (k == "--port") o.port = uint16_t(std::atoi(next().c_str()));
        else if (k == "--queue") o.queueSecs = std::atof(next().c_str());
        else if (k == "-v" || k == "--verbose") o.verbose = true;
        else { usage(); return k == "-h" || k == "--help" ? 0 : 2; }
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
#else
    std::signal(SIGPIPE, SIG_IGN);
#endif
    if (!o.verbose) SoapySDR::setLogLevel(SOAPY_SDR_WARNING);

    Device dev;
    if (!dev.open(o.args)) return 1;
    logf("ionos_spyserver: %s, %.0f sps max, %zu decimation stage(s), %u..%u Hz", dev.label.c_str(),
         dev.rates[0], dev.rates.size() - 1, dev.fmin, dev.fmax);

    sock_t ls = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (ls == SOCK_INVALID) { logf("ionos_spyserver: socket() failed"); return 1; }
    int one = 1;
    ::setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char *>(&one), sizeof one);
    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_port = htons(o.port);
    if (::inet_pton(AF_INET, o.bind.c_str(), &sa.sin_addr) != 1) { logf("ionos_spyserver: bad --bind %s", o.bind.c_str()); return 2; }
    if (::bind(ls, reinterpret_cast<sockaddr *>(&sa), sizeof sa) != 0 || ::listen(ls, 1) != 0) {
        logf("ionos_spyserver: cannot listen on %s:%u", o.bind.c_str(), o.port);
        return 1;
    }
    logf("ionos_spyserver: listening on %s:%u (SpyServer)", o.bind.c_str(), o.port);

    while (!g_quit) {
        fd_set rf;
        FD_ZERO(&rf);
        FD_SET(ls, &rf);
        timeval tv{0, 250000};
        if (::select(int(ls) + 1, &rf, nullptr, nullptr, &tv) <= 0) continue;
        sockaddr_in ca{};
        socklen_t cl = sizeof ca;
        sock_t cs = ::accept(ls, reinterpret_cast<sockaddr *>(&ca), &cl);
        if (cs == SOCK_INVALID) continue;
        ::setsockopt(cs, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char *>(&one), sizeof one);
        char ip[64] = {};
        ::inet_ntop(AF_INET, &ca.sin_addr, ip, sizeof ip);
        logf("ionos_spyserver: client %s connected", ip);
        {
            Session s(dev, cs, o);
            s.run();
        }
        sock_close(cs);
        logf("ionos_spyserver: client %s disconnected", ip);
    }

    sock_close(ls);
    dev.close();
#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
