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
#include <algorithm>
#include <cmath>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sndfile.h>
#include <curl/curl.h>

#ifndef __APPLE__
  #include "webrtc_vad.h"
  #include <alsa/asoundlib.h>
#endif

#include "nova/audio/AlsaCapture.hpp"
#include "nova/audio/doa_gcc_phat.hpp"
#include "nova/audio/PorcupineHandler.hpp"
#include "nova/ServerClient.hpp"

using namespace std::chrono_literals;

// ----------------------- Config -----------------------
namespace cfg {
    // Audio / DoA
    inline const char* kAlsaDevice   = "hw:2,0"; // check with `arecord -l`
    constexpr unsigned kRate         = 48000;
    constexpr unsigned kChannels     = 2;        // L/R (two mics time-division on I2S)
    constexpr snd_pcm_format_t kFormat  = SND_PCM_FORMAT_S32_LE;
    constexpr snd_pcm_uframes_t kPeriodFrames = 480;
    constexpr float kMicSpacingM     = 0.189f;   // 18.9 cm
    constexpr int   kLevelShift      = 8;        // downshift when summing absolute
    constexpr float kGain            = 1.0f;
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

struct SpeechGate {
    int voice_count = 0;
    int silence_count = 0;
    bool speech_active = false;

    // parameters you can tune
    static constexpr int kOnFrames  = 5;   // need 5 voiced frames (~50 ms) to activate
    static constexpr int kOffFrames = 20;  // need 20 silent frames (~200 ms) to deactivate

    bool update(int vad_result) {
        if (vad_result == 1) {
            voice_count++;
            silence_count = 0;
            if (!speech_active && voice_count >= kOnFrames) {
                speech_active = true;
            }
        } else {
            silence_count++;
            voice_count = 0;
            if (speech_active && silence_count >= kOffFrames) {
                speech_active = false;
            }
        }
        return speech_active;
    }
};

struct DcFilter {
    float prev_in = 0.0f;
    float prev_out = 0.0f;

    int16_t process(int16_t sample) {
        constexpr float alpha = 0.995f; // cutoff ~20 Hz
        float x = static_cast<float>(sample);
        float y = alpha * (prev_out + x - prev_in);
        prev_in = x;
        prev_out = y;
        return static_cast<int16_t>(std::clamp(y, -32768.0f, 32767.0f));
    }
};

void save_clip(const std::vector<int16_t>& samples, const std::string& path) {
    SF_INFO sfinfo{};
    sfinfo.samplerate = 48000;
    sfinfo.channels = 1;
    sfinfo.format = SF_FORMAT_WAV | SF_FORMAT_PCM_16;

    SNDFILE* sndfile = sf_open(path.c_str(), SFM_WRITE, &sfinfo);
    if (!sndfile) {
        std::cerr << "[save_clip] Failed to open file: " << sf_strerror(nullptr) << std::endl;
        return;
    }

    sf_count_t written = sf_write_short(sndfile, samples.data(), samples.size());
    if (written != static_cast<sf_count_t>(samples.size())) {
        std::cerr << "[save_clip] Incomplete write: " << written << " of " << samples.size() << " samples\n";
    }

    sf_close(sndfile);
    std::cout << "[save_clip] Saved " << path << " (" << samples.size() << " samples)\n";
}

void upload_clip(const std::vector<int16_t>& samples) {
    const char* filename = "wake_clip.wav";
    save_clip(samples, filename);  // First save it locally

    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "[upload_clip] Failed to init curl\n";
        return;
    }

    curl_mime* form = curl_mime_init(curl);
    curl_mimepart* part = curl_mime_addpart(form);
    curl_mime_name(part, "file");
    curl_mime_filedata(part, filename);

    curl_easy_setopt(curl, CURLOPT_URL, "https://nova-server-iy13.onrender.com/process_audio");
    curl_easy_setopt(curl, CURLOPT_MIMEPOST, form);

    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        std::cerr << "[upload_clip] curl error: " << curl_easy_strerror(res) << std::endl;
    } else {
        std::cout << "[upload_clip] Upload successful\n";
    }

    curl_mime_free(form);
    curl_easy_cleanup(curl);
}

// Minimal event bus / state machine
enum class State { CHARGING, IDLE, LISTENING, SPEAKING };
enum class Ev { IdleTick, ScanTick, WakeWord, TtsStarted, TtsFinished, ChargeStarted, ChargeStopped };
struct Event { Ev id; };

std::mutex g_q_m;
std::condition_variable g_q_cv;
std::queue<Event> g_q;

std::atomic<bool>  g_quit{false};

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
    SpeechGate gate;
    std::deque<int16_t> prebuffer;   // ~1 sec of audio
    bool recording = false;
    std::vector<int16_t> current_clip;

    try {
        std::cout << "[thread] audio_doa_thread started" << std::endl;

        // === ALSA device ===
        AlsaCapture alsa(cfg::kAlsaDevice,
                         cfg::kRate,
                         cfg::kChannels,
                         cfg::kFormat,
                         cfg::kPeriodFrames);

        // === VAD ===
        VadInst* vad = WebRtcVad_Create();
        WebRtcVad_Init(vad);
        WebRtcVad_set_mode(vad, 3);

        // === Porcupine init (assume already created globally) ===
        extern pv_porcupine_t *porcupine;
        static std::vector<int16_t> porcupine_buf; // accumulates 16kHz samples
        PorcupineHandler porcupine(
            "../external/porcupine/lib/common/porcupine_params.pv",  // model path
            "../models/Hey-Nova_en_raspberry-pi_v3_0_0.ppn"          // your wakeword file
        );

        // === Buffers ===
        std::vector<int32_t> buf(cfg::kPeriodFrames * cfg::kChannels);
        std::vector<int16_t> mono;
        mono.reserve(cfg::kPeriodFrames);

        DcFilter dc_filter;
        std::vector<int16_t> frame480;
        frame480.reserve(480);

        while (!g_quit.load()) {
            const size_t got = alsa.read(buf);
            if (got == 0) continue;

            // Stereo → mono + DC remove + gain
            mono.resize(got / cfg::kChannels);
            for (size_t i = 0; i < mono.size(); i++) {
                int32_t left = buf[2 * i];
                int32_t right = buf[2 * i + 1];
                int32_t sample = (left + right) / 2;

                int16_t shifted = static_cast<int16_t>(sample >> 14);
                int16_t filtered = dc_filter.process(shifted);
                int32_t amplified = static_cast<int32_t>(filtered * cfg::kGain);
                mono[i] = std::clamp(amplified, -32768, 32767);
            }

            // Break into 480-sample frames
            for (auto s : mono) {
                frame480.push_back(s);
                if (frame480.size() == 480) {
                    // --- Debug stats ---
                    long long sum = 0;
                    int16_t minv = 32767, maxv = -32768;
                    for (auto v : frame480) {
                        sum += v * v;
                        minv = std::min(minv, v);
                        maxv = std::max(maxv, v);
                    }
                    float rms = std::sqrt(sum / 480.0f);

                    // --- VAD ---
                    int vad_result = WebRtcVad_Process(vad, cfg::kRate,
                                                       frame480.data(), frame480.size());
                    bool speech_detected = gate.update(vad_result);

                    // === Downsample to 16kHz for Porcupine ===
                    std::vector<int16_t> frame16k;
                    frame16k.reserve(160);
                    for (size_t i = 0; i < frame480.size(); i += 3) {
                        frame16k.push_back(frame480[i]);
                    }
                    porcupine_buf.insert(porcupine_buf.end(),
                                         frame16k.begin(), frame16k.end());

                    while (!recording && porcupine_buf.size() >= 512) {
                        int32_t keyword_index = -1;
                        pv_status_t status = pv_porcupine_process(
                            porcupine,
                            porcupine_buf.data(),
                            &keyword_index);

                        // pop the 512 we just processed
                        porcupine_buf.erase(porcupine_buf.begin(),
                                            porcupine_buf.begin() + 512);

                        if (status == PV_STATUS_SUCCESS && keyword_index >= 0) {
                            // Wake word detected → start recording
                            std::vector<int16_t> candidate(prebuffer.begin(), prebuffer.end());
                            candidate.insert(candidate.end(), frame480.begin(), frame480.end());

                            recording = true;
                            current_clip = candidate;

                            float angle_deg = 0.0f, confidence = 0.0f;
                            confidence = nova::audio::estimate_direction(buf.data(),
                                                                         got / cfg::kChannels,
                                                                         cfg::kRate,
                                                                         cfg::kMicSpacingM,
                                                                         &angle_deg);

                            std::cout << "[wakeword] doa=" << angle_deg
                                      << " conf=" << confidence << std::endl;
                        }
                    }

                    if (recording) {
                        current_clip.insert(current_clip.end(), frame480.begin(), frame480.end());

                        if (!speech_detected) {
                            save_clip(current_clip, "wake_clip.wav");
                            upload_clip(current_clip);
                            recording = false;
                            current_clip.clear();
                        }
                    }

                    // maintain rolling prebuffer
                    prebuffer.insert(prebuffer.end(),
                                     frame480.begin(), frame480.end());
                    while (prebuffer.size() > cfg::kRate) {
                        prebuffer.erase(prebuffer.begin(),
                                        prebuffer.begin() + frame480.size());
                    }

                    frame480.clear();
                }
            }
        }

        WebRtcVad_Free(vad);
        std::cout << "[thread] audio_doa_thread exiting" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[thread] exception: " << e.what() << std::endl;
    }
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
