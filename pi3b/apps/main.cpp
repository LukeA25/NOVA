// Copyright (c) 2025 Luke Anderson – MIT License

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
#include <fstream>
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
#include <speex/speex_resampler.h>

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
    constexpr float kGain            = 5.0f;
    constexpr int64_t kLevelThresh   = 5e6;      // crude gate; calibrate later
    constexpr float kConfThresh      = 0.10f;    // crude conf gate; calibrate later
    inline constexpr int porcupine_frame_length = 160;
    const int kVadTailFrames         = 20;

    // UART to Pico
    inline const char* kUartDev      = "/dev/serial0";
    constexpr speed_t  kUartBaud     = B115200;

    // Wi-Fi / UDP
    inline const char* kUdpTargetIp = "10.20.0.178"; // <-- set to Zero 2 W IP
    constexpr uint16_t kUdpTxPort     = 5005; // Outgoing telemetry
    constexpr uint16_t kUdpRxPort   = 5006; // Incoming control

    // Timers
    constexpr auto kIdleTickPeriod   = 30s;
    constexpr auto kScanTickPeriod   = 5min;
}
// ------------------------------------------------------

// ----------------------- Pico UART Structs -----------------------
#pragma pack(push,1)

typedef enum {
    EASE_NONE,
    EASE_IN,
    EASE_OUT,
    EASE_IN_OUT,
} EaseMode_t;

typedef struct {
    float      target_deg;
    uint16_t   time_to_complete_ms;
    EaseMode_t ease;
} ServoTarget_t;

typedef struct { float yaw, pitch, roll; } Euler3f;

typedef struct {
    Euler3f    target_pos;
    uint16_t   time_to_complete_ms;
} HeadTarget_t;

typedef struct {
    HeadTarget_t  head;
    ServoTarget_t servos[3];
    uint16_t      time_to_complete_ms;
} FullCommandFrame_t;

struct Animation {
    std::vector<FullCommandFrame_t> frames;
};

#pragma pack(pop)
// ------------------------------------------------------

// ------------------- Animations -------------------
static Animation listen_pose = {
    .frames = {
        {
            .head = {{ .yaw = 0.0f, .pitch = 0.0f, .roll = 0.0f }, .time_to_complete_ms = 3000},
            .servos = {
                { .target_deg = 120.0f, .time_to_complete_ms = 3000, .ease = EASE_IN },
                { .target_deg = 60.0f, .time_to_complete_ms = 3000, .ease = EASE_IN },
                { .target_deg = 100.0f, .time_to_complete_ms = 3000, .ease = EASE_IN },
            },
            .time_to_complete_ms = 3000
        },
    } 
};

static Animation sleep_animation = {
    .frames = {
        {
            .head = {{ .yaw = 0.0f, .pitch = 0.0f, .roll = 0.0f }, .time_to_complete_ms = 3000},
            .servos = {
                { .target_deg = 120.0f, .time_to_complete_ms = 3000, .ease = EASE_IN },
                { .target_deg = 60.0f, .time_to_complete_ms = 3000, .ease = EASE_IN },
                { .target_deg = 100.0f, .time_to_complete_ms = 3000, .ease = EASE_IN },
            },
            .time_to_complete_ms = 3000
        },
        {
            .head = {{ .yaw = 0.0f, .pitch = 0.0f, .roll = 0.0f }, .time_to_complete_ms = 2000},
            .servos = {
                { .target_deg = 75.0f, .time_to_complete_ms = 2000, .ease = EASE_OUT },
                { .target_deg = 60.0f, .time_to_complete_ms = 2000, .ease = EASE_OUT },
                { .target_deg = 120.0f, .time_to_complete_ms = 2000, .ease = EASE_OUT },
            },
            .time_to_complete_ms = 2000
        }
    }
};

static Animation wake_animation = {
    .frames = {
        {
            .head = {{ .yaw = 0.0f, .pitch = 0.0f, .roll = 0.0f }, .time_to_complete_ms = 500},
            .servos = {
                { .target_deg = 75.0f, .time_to_complete_ms = 500, .ease = EASE_IN },
                { .target_deg = 60.0f, .time_to_complete_ms = 500, .ease = EASE_IN },
                { .target_deg = 100.0f, .time_to_complete_ms = 500, .ease = EASE_IN },
            },
            .time_to_complete_ms = 500
        },
        {
            .head = {{ .yaw = 0.0f, .pitch = 0.0f, .roll = 0.0f }, .time_to_complete_ms = 1000},
            .servos = {
                { .target_deg = 100.0f, .time_to_complete_ms = 1000, .ease = EASE_IN },
                { .target_deg = 60.0f, .time_to_complete_ms = 1000, .ease = EASE_IN },
                { .target_deg = 100.0f, .time_to_complete_ms = 1000, .ease = EASE_NONE },
            },
            .time_to_complete_ms = 1000
        },
        {
            .head = {{ .yaw = 0.0f, .pitch = 0.0f, .roll = 0.0f }, .time_to_complete_ms = 3000},
            .servos = {
                { .target_deg = 135.0f, .time_to_complete_ms = 2000, .ease = EASE_OUT },
                { .target_deg = 125.0f, .time_to_complete_ms = 2000, .ease = EASE_OUT },
                { .target_deg = 50.0f, .time_to_complete_ms = 2000, .ease = EASE_OUT },
            },
            .time_to_complete_ms = 2000
        }
    }
};

static std::vector<Animation> idle_animations = {
    // First Animation
    {
        {
            // frames
            {
                .head = {{0.0f, 0.0f, 0.0f}, 1000},
                .servos = {
                    {90.0f, 1000, EASE_IN},
                    {45.0f, 1000, EASE_OUT},
                    {30.0f, 1000, EASE_IN_OUT}
                },
                .time_to_complete_ms = 1000
            },
            {
                .head = {{5.0f, 0.0f, 0.0f}, 800},
                .servos = {
                    {100.0f, 800, EASE_OUT},
                    {50.0f, 800, EASE_IN},
                    {35.0f, 800, EASE_NONE}
                },
                .time_to_complete_ms = 800
            }
        }
    },

    // Second Animation
    {
        {
            {
                .head = {{-5.0f, 3.0f, 0.0f}, 1200},
                .servos = {
                    {70.0f, 1200, EASE_IN_OUT},
                    {60.0f, 1200, EASE_IN},
                    {40.0f, 1200, EASE_OUT}
                },
                .time_to_complete_ms = 1200
            },
            {
                .head = {{0.0f, 0.0f, 0.0f}, 1000},
                .servos = {
                    {90.0f, 1000, EASE_NONE},
                    {45.0f, 1000, EASE_NONE},
                    {30.0f, 1000, EASE_NONE}
                },
                .time_to_complete_ms = 1000
            }
        }
    }
};
// ------------------------------------------------------

// ----------------------- Audio Helper Functions -----------------------
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

struct SpeechGate {
    int voice_count = 0;
    int silence_count = 0;
    bool speech_active = false;

    static constexpr int kOnFrames  = 5;   // need 5 voiced frames (~50 ms) to activate
    static constexpr int kOffFrames = 50;  // need 20 silent frames (~200 ms) to deactivate

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

void save_clip(const std::vector<int16_t>& samples, const std::string& path, int rate = 48000, int channels = 1) {
    SF_INFO sfinfo{};
    sfinfo.samplerate = rate;
    sfinfo.channels = channels;
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
    const char* input_filename = "wake_clip.wav";
    const char* output_filename = "response.mp3";
    const char* output_final_filename = "response_clip.mp3";

    // Save the input audio file
    save_clip(samples, input_filename);
    std::cout << "[upload_clip] Saved input to: " << input_filename << "\n";

    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "[upload_clip] Failed to init curl\n";
        return;
    }

    // Open output file to write the response audio
    FILE* out = fopen(output_filename, "wb");
    if (!out) {
        std::cerr << "[upload_clip] Failed to open output file for writing\n";
        curl_easy_cleanup(curl);
        return;
    }

    // Prepare multipart form
    curl_mime* form = curl_mime_init(curl);
    curl_mimepart* part = curl_mime_addpart(form);
    curl_mime_name(part, "file");
    curl_mime_filedata(part, input_filename);

    // Set options
    curl_easy_setopt(curl, CURLOPT_URL, "https://nova-server-iy13.onrender.com/process_audio");
    curl_easy_setopt(curl, CURLOPT_MIMEPOST, form);

    // Write server response (audio file) to output file
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, NULL);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, out);

    std::cout << "[upload_clip] Uploading to server...\n";
    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        std::cerr << "[upload_clip] curl error: " << curl_easy_strerror(res) << "\n";
    } else {
        std::cout << "[upload_clip] Upload and download successful. Response saved to: " << output_filename << "\n";

        if (std::rename(output_filename, output_final_filename) != 0) {
            std::cerr << "[upload_clip] Failed to rename file to " << output_final_filename << "\n";
        } else {
            std::cout << "[upload_clip] Renamed to: " << output_final_filename << "\n";
        }
    }

    fclose(out);
    curl_mime_free(form);
    curl_easy_cleanup(curl);
}
// ------------------------------------------------------

// ------------------- State Machine Setup -------------------
enum class State { CHARGING, IDLE, LISTENING, SPEAKING };
enum class Ev { IdleTick, ScanTick, WakeWord, TtsStarted, TtsFinished, ChargeStarted, ChargeStopped };
struct Event { Ev id; };

static State state = State::IDLE;

std::mutex g_q_m;
std::condition_variable g_q_cv;
std::queue<Event> g_q;

std::atomic<bool>  g_quit{false};

std::queue<FullCommandFrame_t> uart_queue;
std::mutex uart_q_m;
std::condition_variable uart_q_cv;

std::mutex audio_mtx;
std::condition_variable audio_cv;
std::atomic<bool> audio_active{true};

static void push(Event e) {
    std::lock_guard<std::mutex> lk(g_q_m);
    g_q.push(e);
    g_q_cv.notify_one();
}

static void on_sigint(int) { 
    g_quit.store(true);
    g_q_cv.notify_all();
    uart_q_cv.notify_all();
    audio_cv.notify_all();
}
// ------------------------------------------------------
// ------------------- Producers -------------------
void audio_doa_thread() {
    SpeechGate gate;
    std::deque<int16_t> prebuffer;
    bool recording = false;
    bool wake_detected = false;
    SpeexResamplerState *resampler = nullptr;

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

        // === Porcupine Init ===
        nova::PorcupineHandler porcupine(
	        "hsbr8a0ahcO/A8E+4iABuOkYDSTA17VcQgtDLxggsC6twCiKQAFLLA==",
            "/home/nova/NOVA/pi3b/external/porcupine/lib/common/porcupine_params.pv",
            "/home/nova/NOVA/pi3b/models/Hey-Nova_en_raspberry-pi_v3_0_0.ppn"
        );

        // === Speex Init ===
        int err;
        resampler = speex_resampler_init(
            1,          // 1 channel (mono)
            48000,      // input rate (your source audio)
            16000,      // output rate (Porcupine expects this)
            5,          // quality level (0 = worst, 10 = best)
            &err
        );

        if (err != RESAMPLER_ERR_SUCCESS || resampler == nullptr) {
            std::cerr << "Failed to init Speex resampler: " << speex_resampler_strerror(err) << std::endl;
            return;
        }

        // --- Buffers ---
        std::vector<int32_t> buf(cfg::kPeriodFrames * cfg::kChannels);
        std::vector<int16_t> mono;
        mono.reserve(cfg::kPeriodFrames);

        DcFilter dc_filter;
        std::vector<int16_t> frame480;
        frame480.reserve(480);

        std::vector<int16_t> porcupine_frame;
        porcupine_frame.reserve(700);
        std::vector<int16_t> clip_buffer;

        while (!g_quit.load()) {
            std::unique_lock<std::mutex> lock(audio_mtx);
            audio_cv.wait(lock, [] { return audio_active.load() || g_quit.load(); });
            lock.unlock();
            if (g_quit.load()) break;

            const size_t got = alsa.read(buf);
            if (got == 0) continue;

            mono.resize(got);
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
                    // --- VAD ---
                    int vad_result = WebRtcVad_Process(vad, cfg::kRate,
                                                       frame480.data(),
                                                       frame480.size());
                    bool speech_detected = gate.update(vad_result);

                    std::vector<int16_t> downsampled_16k(160);
                    spx_uint32_t in_len = 480;
                    spx_uint32_t out_len = 160;
                    speex_resampler_process_int(resampler, 0,
                                                frame480.data(), &in_len,
                                                downsampled_16k.data(), &out_len);

                    porcupine_frame.insert(porcupine_frame.end(),
                                           downsampled_16k.begin(),
                                           downsampled_16k.begin() + out_len);

                    // Process complete frames
                    while (porcupine_frame.size() >= 512) {
                        int keyword_index = porcupine.process_frame(porcupine_frame.data());

                        if (keyword_index >= 0) {
                            push({Ev::WakeWord});
                            std::cout << "[Wake Word] Detected index " << keyword_index << std::endl;
                            wake_detected = true;
                            recording = true;
                            clip_buffer.clear();
                        }

                        porcupine_frame.erase(porcupine_frame.begin(),
                        porcupine_frame.begin() + 512);
                    }

                    if (recording) {
                        clip_buffer.insert(clip_buffer.end(), frame480.begin(), frame480.end());
                    }

                    if (!speech_detected && recording) {
                        std::cout << "Speech ended, saving clip..." << std::endl;
                        save_clip(clip_buffer, "voice_clip.wav");
                        upload_clip(clip_buffer);
                        clip_buffer.clear();
                        recording = false;
                        wake_detected = false;
                    }

                    frame480.clear();
                }
            }
        }

        WebRtcVad_Free(vad);
        speex_resampler_destroy(resampler);
        std::cout << "[thread] audio_doa_thread exiting" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[thread] exception: " << e.what() << std::endl;
    }
}

int open_serial(const char* device, int baud) {
    int fd = ::open(device, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        perror("Failed to open UART");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

void uart_thread() {
    int fd = open_serial(cfg::kUartDev, cfg::kUartBaud);
    if (fd < 0) return;

    while (!g_quit.load()) {
        FullCommandFrame_t cmd;
        {
            std::unique_lock<std::mutex> lk(uart_q_m);
            uart_q_cv.wait(lk, []{ return !uart_queue.empty() || g_quit.load(); });
            if (g_quit.load()) break;
            cmd = uart_queue.front();
            uart_queue.pop();
        }

        uint8_t buffer[31];
        buffer[0] = 0xAB;
        buffer[1] = 0xCD;

        memcpy(&buffer[2],  &cmd.head.target_pos.pitch, 4);
        memcpy(&buffer[6],  &cmd.head.target_pos.roll,  4);
        memcpy(&buffer[10], &cmd.head.target_pos.yaw,   4);

        buffer[14] = cmd.head.time_to_complete_ms & 0xFF;
        buffer[15] = (cmd.head.time_to_complete_ms >> 8) & 0xFF;

        for (int i = 0; i < 3; ++i) {
            int offset = 16 + i * 5;
            uint16_t deg = cmd.servos[i].target_deg;
            uint16_t time = cmd.servos[i].time_to_complete_ms;
            uint8_t ease = cmd.servos[i].ease;

            buffer[offset + 0] = deg & 0xFF;
            buffer[offset + 1] = (deg >> 8) & 0xFF;
            buffer[offset + 2] = time & 0xFF;
            buffer[offset + 3] = (time >> 8) & 0xFF;
            buffer[offset + 4] = ease;
        }

        ssize_t n = write(fd, buffer, 31);
        if (n != sizeof(cmd)) {
            perror("UART write");
        } else {
            std::cout << "[UART TX] Sent " << n << " bytes" << std::endl;
        }
    }

    ::close(fd);
}

void wifi_tx_thread() {
    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        return;
    }

    timeval tv{.tv_sec = 0, .tv_usec = 20000}; // 20ms timeout
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(cfg::kUdpTxPort);
    dst.sin_addr.s_addr = inet_addr(cfg::kUdpTargetIp);

    const std::string filepath = "response_clip.mp3";
    const size_t kMaxChunkSize = 1024;

    while (!g_quit.load()) {
        if (std::ifstream test(filepath); !test.good()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        std::ifstream file(filepath, std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "[wifi_tx] Failed to open file: " << filepath << "\n";
            break;
        }

        std::cout << "[wifi_tx] Sending file: " << filepath << "\n";

        std::vector<char> buffer(kMaxChunkSize);
        while (!file.eof()) {
            file.read(buffer.data(), buffer.size());
            std::streamsize bytes_read = file.gcount();

            if (bytes_read > 0) {
                ssize_t sent = sendto(sock, buffer.data(), bytes_read, 0,
                                      (sockaddr*)&dst, sizeof(dst));
                if (sent < 0) {
                    perror("[wifi_tx] sendto");
                    break;
                }
            }
        }

        file.close();
        std::cout << "[wifi_tx] File sent successfully.\n";

        std::remove(filepath.c_str());

        const char* end_signal = "END";
        ssize_t sent = sendto(sock, end_signal, 3, 0, (sockaddr*)&dst, sizeof(dst));
        if (sent < 0) {
            perror("[wifi_tx] Failed to send END signal");
        } else {
            std::cout << "[wifi_tx] Sent END signal\n";
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

void idle_animator_thread() {
    using namespace std::chrono_literals;

    while (g_quit.load() == false) {
        if (state != State::IDLE) {
            std::this_thread::sleep_for(100ms);
            continue;
        }

        static size_t anim_index = 0;
        Animation& anim = idle_animations[anim_index];

        std::this_thread::sleep_for(10s + std::chrono::seconds(rand() % 20));

        if (state != State::IDLE) continue;

        for (auto& frame : anim.frames) {
            if (state != State::IDLE) break;

            uart_queue.push(frame);
            std::this_thread::sleep_for(std::chrono::duration<float>(frame.time_to_complete_ms));
        }

        anim_index = (anim_index + 1) % idle_animations.size();
    }
}
// ------------------------------------------------------

// ------------------- Consumer (State Machine) -------------------
int main() {
    std::signal(SIGINT, on_sigint);

    std::thread(audio_doa_thread).detach();
    std::thread(uart_thread).detach();
    std::thread(wifi_tx_thread).detach();
    std::thread(wifi_rx_thread).detach();
    // std::thread(idle_animator_thread).detach();

    while (!g_quit.load()) {
        Event ev;
        {
            std::unique_lock<std::mutex> lk(g_q_m);
            g_q_cv.wait(lk, []{ return !g_q.empty() || g_quit.load(); });
            if (g_quit.load()) break;
            ev = g_q.front(); g_q.pop();
        }

        switch (state) {
        case State::CHARGING:
            if (ev.id == Ev::ChargeStopped) {
                state = State::IDLE;
                {
                    std::lock_guard<std::mutex> lock(audio_mtx);
                    audio_active = true;
                }
                audio_cv.notify_one();
                std::cout << "[state] -> IDLE (charge stopped)\n";

                for (const FullCommandFrame_t& frame : wake_animation.frames) {
                    {
                        std::lock_guard<std::mutex> lk(uart_q_m);
                        uart_queue.push(frame);
                    }
                    uart_q_cv.notify_one();
                    std::this_thread::sleep_for(std::chrono::milliseconds(frame.time_to_complete_ms));
                }

                {
                    std::lock_guard<std::mutex> lk(uart_q_m);
                    while (!uart_queue.empty()) uart_queue.pop();  // Clear queue ONCE
                }

                break;
            }
            break;

        case State::IDLE:
            if (ev.id == Ev::ChargeStarted) {
                state = State::CHARGING;
                audio_active = false;
                std::cout << "[state] -> CHARGING\n";

                for (const FullCommandFrame_t& frame : sleep_animation.frames) {
                    {
                        std::lock_guard<std::mutex> lk(uart_q_m);
                        uart_queue.push(frame);
                    }
                    uart_q_cv.notify_one();

                    std::this_thread::sleep_for(std::chrono::milliseconds(frame.time_to_complete_ms));
                }

                {
                    std::lock_guard<std::mutex> lk(g_q_m);
                    while (!g_q.empty()) g_q.pop();
                }
                break;
            } else if (ev.id == Ev::WakeWord) {
                {
                    std::lock_guard<std::mutex> lk(uart_q_m);
                    uart_queue.push(listen_pose.frames[0]);
                }
                uart_q_cv.notify_one();

                break;
            }
            break;

        case State::LISTENING:
            if (ev.id == Ev::ChargeStarted) {
                state = State::CHARGING;
                std::cout << "[state] -> CHARGING\n";
                break;
            }
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
