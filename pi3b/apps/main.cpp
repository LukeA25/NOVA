#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <mutex>
#include <queue>
#include <cstring>
#include <thread>
#include <iostream>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#ifndef __APPLE__
  #include <webrtc_vad/webrtc_vad.h>
  #include <alsa/asoundlib.h>
#endif

#include "nova/audio/AlsaCapture.hpp"
#include "nova/audio/doa_gcc_phat.hpp"
#include "nova/ServerClient.hpp"

using namespace std::chrono_literals;

// ----------------------- Config -----------------------
namespace cfg {
    // Audio / DoA
    inline const char* kAlsaDevice   = "hw:2,0"; // check with `arecord -l`
    constexpr unsigned kRate         = 48000;
    constexpr unsigned kChannels     = 2;        // L/R (two mics time-division on I2S)
    constexpr snd_pcm_format_t kFmt  = SND_PCM_FORMAT_S32_LE;
    constexpr snd_pcm_uframes_t kPeriodFrames = 256;
    constexpr float kMicSpacingM     = 0.189f;   // 18.9 cm
    constexpr int   kLevelShift      = 8;        // downshift when summing absolute
    constexpr int64_t kLevelThresh   = 5e6;      // crude gate; calibrate later
    constexpr float kConfThresh      = 0.10f;    // crude conf gate; calibrate later

    // UART to Pico
    inline const char* kUartDev      = "/dev/serial0";
    constexpr speed_t  kUartBaud     = B115200;

	// Wi-Fi / UDP
    inline const char* kUdpTargetIp = "192.168.1.42"; // <-- set to Zero 2 W IP
    constexpr uint16_t kUdpTxPort     = 5005; // Outgoing telemetry
    constexpr uint16_t kUdpRxPort   = 5006; // Incoming control

    // Timers
    constexpr auto kIdleTickPeriod   = 30s;
    constexpr auto kScanTickPeriod   = 5min;
}
// ------------------------------------------------------

// Minimal event bus / state machine
enum class State { CHARGING, IDLE, LISTENING, SPEAKING };
enum class Ev { IdleTick, ScanTick, WakeWord, TtsStarted, TtsFinished, ChargeStarted, ChargeStopped };
struct Event { Ev id; };

std::mutex g_q_m;
std::condition_variable g_q_cv;
std::queue<Event> g_q;

std::atomic<bool>  g_quit{false};
std::atomic<float> g_doa_deg{0.0f};   // latest DoA estimate (deg)
std::atomic<float> g_doa_conf{0.0f};  // latest PHAT peak as crude “confidence”

static void push(Event e) {
    std::lock_guard<std::mutex> lk(g_q_m);
    g_q.push(e);
    g_q_cv.notify_one();
}

static void on_sigint(int){ g_quit.store(true); }

// ------------------- Producers -------------------

void idle_timer() {
    while (!g_quit.load()) { std::this_thread::sleep_for(cfg::kIdleTickPeriod); push({Ev::IdleTick}); }
}

void scan_timer() {
    while (!g_quit.load()) { std::this_thread::sleep_for(cfg::kScanTickPeriod); push({Ev::ScanTick}); }
}


void audio_doa_thread() {
#ifndef __APPLE__
    AlsaCapture cap(cfg::kAlsaDevice, cfg::kRate, cfg::kChannels, cfg::kFmt, cfg::kPeriodFrames);
    std::vector<int32_t> buf;

    // Setup VAD
    VadInst* vad = WebRtcVad_Create();
    WebRtcVad_Init(vad);
    WebRtcVad_set_mode(vad, 3);

    const int frame_size = 480; // 10 ms @ 48kHz

    // Init server client
    ServerClient client("https://your-render-server.onrender.com/asr");

    while (!g_quit.load()) {
        snd_pcm_sframes_t got = cap.read(buf);
        if (got <= 0) continue;

        // Convert to mono int16 for VAD
        std::vector<int16_t> mono(got);
        for (snd_pcm_sframes_t i = 0; i < got; i++) {
            int64_t l = buf[2*i];
            int64_t r = buf[2*i+1];
           [48;52;171;2080;3420t mono[i] = static_cast<int16_t>((l + r) >> 17);
        }

        bool speech_detected = false;
        for (size_t i = 0; i + frame_size <= mono.size(); i += frame_size) {
            int vad_res = WebRtcVad_Process(vad, cfg::kRate, mono.data() + i, frame_size);
            if (vad_res == 1) {
                speech_detected = true;
                break;
            }
        }

        if (speech_detected) {
            // --- Save buffer to WAV ---
            const char* path = "/tmp/utterance.wav";

            SF_INFO sfinfo{};
            sfinfo.channels = 2;                // stereo
            sfinfo.samplerate = cfg::kRate;     // 48000
            sfinfo.format = SF_FORMAT_WAV | SF_FORMAT_PCM_32;

            SNDFILE* outfile = sf_open(path, SFM_WRITE, &sfinfo);
            if (!outfile) {
                std::cerr << "Failed to open WAV file for writing\n";
                continue;
            }

            sf_write_int(buf.data(), buf.size(), outfile);
            sf_close(outfile);

            // --- Upload to server ---
            std::string response;
            if (client.upload_wav(path, response)) {
                std::cout << "[upload] Server response: " << response << "\n";
            } else {
                std::cerr << "[upload] Failed to send file\n";
            }
        }
    }

    WebRtcVad_Free(vad);
#endif
}

int open_serial(const char* dev, speed_t baud) {
    int fd = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror("open"); std::exit(1); }

    termios tio{};
    if (tcgetattr(fd, &tio) < 0) { perror("tcgetattr"); std::exit(1); }

    cfmakeraw(&tio);
    cfsetispeed(&tio, baud);
    cfsetospeed(&tio, baud);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~PARENB;  // 8N1
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN]  = 0;     // non-blocking read
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tio) < 0) { perror("tcsetattr"); std::exit(1); }
    return fd;
}

// Sends the *live DoA* to the Pico over UART every 200 ms
void uart_thread() {
    int fd = open_serial(cfg::kUartDev, cfg::kUartBaud);

    while (!g_quit.load()) {
        float deg  = g_doa_deg.load(std::memory_order_relaxed);
        float conf = g_doa_conf.load(std::memory_order_relaxed);

        char line[64];
        int n = std::snprintf(line, sizeof(line), "DOA:%+.1f,CONF:%.3f\n", deg, conf);
        if (n > 0) (void)::write(fd, line, n);

        std::this_thread::sleep_for(200ms);
    }
    ::close(fd);
}

void wifi_tx_thread() {
    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return; }

    timeval tv{.tv_sec = 0, .tv_usec = 20000};
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port   = htons(cfg::kUdpTxPort);
    dst.sin_addr.s_addr = inet_addr(cfg::kUdpTargetIp);

    while (!g_quit.load()) {
        float deg  = g_doa_deg.load(std::memory_order_relaxed);
        float conf = g_doa_conf.load(std::memory_order_relaxed);

        char msg[128];
        int n = std::snprintf(msg, sizeof(msg),
                              "{\"type\":\"doa\",\"deg\":%.1f,\"conf\":%.3f}\n",
                              deg, conf);
        if (n > 0) {
            (void)::sendto(sock, msg, (size_t)n, 0,
                           reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    ::close(sock);
}

void wifi_rx_thread() {
    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return; }

    // Allow quick rebinding after restarts
    int yes = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(cfg::kUdpRxPort);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("bind");
        ::close(sock);
        return;
    }

    // Timeout so we can exit promptly
    timeval tv{.tv_sec = 0, .tv_usec = 200000}; // 200 ms
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    char buf[512];
    while (!g_quit.load()) {
        ssize_t n = ::recv(sock, buf, sizeof(buf)-1, 0);
        if (n <= 0) continue;
        buf[n] = 0;

        if (std::strstr(buf, "\"type\":\"charge\"")) {
            if (std::strstr(buf, "\"state\":\"start\"")) {
                push({Ev::ChargeStarted});
            } else if (std::strstr(buf, "\"state\":\"stop\"")) {
                push({Ev::ChargeStopped});
            }
        }
    }
    ::close(sock);
}

// ------------------- Consumer (state machine) -------------------

int main() {
    std::signal(SIGINT, on_sigint);

    std::thread(idle_timer).detach();
    std::thread(scan_timer).detach();
    std::thread(audio_doa_thread).detach();
    std::thread(uart_thread).detach();
	std::thread(wifi_tx_thread).detach();
	std::thread(wifi_rx_thread).detach();

    State state = State::IDLE;

    auto do_idle_animation = []{
        // TODO: trigger a short, non-blocking idle animation (motors/LEDs)
        std::cout << "[idle] animation\n";
    };

    auto do_visual_scan = []{
        // TODO: quick scan (e.g., camera sweep)
        std::cout << "[idle] visual scan\n";
    };

    auto begin_listening = []{
        // TODO: start VAD/ASR pipeline; on TTS request push Ev::TtsStarted/Ev::TtsFinished
        std::cout << "[listening] start\n";
    };

    while (!g_quit.load()) {
        Event ev;
        {
            std::unique_lock<std::mutex> lk(g_q_m);
            g_q_cv.wait(lk, []{ return !g_q.empty(); });
            ev = g_q.front(); g_q.pop();
        }

        switch (state) {
			case State::CHARGING:
				if (ev.id == Ev::ChargeStopped) {
					state = State::IDLE;
					std::cout << "[state] -> IDLE (charge stopped)\n";
				}
				break;

			case State::IDLE:
				if (ev.id == Ev::ChargeStarted) {
					state = State::CHARGING;
					std::cout << "[state] -> CHARGING\n";
					break;
				}
				// existing IdleTick / ScanTick / WakeWord handling…
				// ...
				break;

			case State::LISTENING:
				if (ev.id == Ev::ChargeStarted) {
					state = State::CHARGING;
					std::cout << "[state] -> CHARGING\n";
					break;
				}
				// existing listening handling…
				break;

			case State::SPEAKING:
				if (ev.id == Ev::ChargeStarted) {
					state = State::CHARGING;
					std::cout << "[state] -> CHARGING\n";
					break;
				}
				if (ev.id == Ev::TtsFinished) {
					state = State::IDLE;
				}
				break;
		}
    }

    std::cout << "Exiting...\n";
    return 0;
}
