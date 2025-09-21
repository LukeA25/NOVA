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
    inline const char* kUdpTargetIp = "192.168.1.42"; // <-- set to Zero 2 W IP
    constexpr uint16_t kUdpTxPort     = 5005; // Outgoing telemetry
    constexpr uint16_t kUdpRxPort   = 5006; // Incoming control

    // Timers
    constexpr auto kIdleTickPeriod   = 30s;
    constexpr auto kScanTickPeriod   = 5min;
}
// ------------------------------------------------------

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
    sfinfo.samplerate = 16000;
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
    const char* input_filename = "wake_clip.wav";
    const char* output_filename = "response_clip.wav";

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
    }

    fclose(out);
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

bool write_wav_file(const std::string& path, const std::vector<int16_t>& data, int channels, int sample_rate) {
    SF_INFO sfinfo;
    sfinfo.frames = data.size() / channels;
    sfinfo.samplerate = sample_rate;
    sfinfo.channels = channels;
    sfinfo.format = SF_FORMAT_WAV | SF_FORMAT_PCM_16;

    SNDFILE* outfile = sf_open(path.c_str(), SFM_WRITE, &sfinfo);
    if (!outfile) {
        std::cerr << "Failed to open output file: " << sf_strerror(nullptr) << std::endl;
        return false;
    }

    sf_count_t written = sf_write_short(outfile, data.data(), data.size());
    if (written != static_cast<sf_count_t>(data.size())) {
        std::cerr << "Failed to write all samples: wrote " << written << "/" << data.size() << std::endl;
        sf_close(outfile);
        return false;
    }

    sf_close(outfile);
    return true;
}

struct SpeechGate {
    int voice_count = 0;
    int silence_count = 0;
    bool speech_active = false;

    // parameters you can tune
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

void audio_doa_thread() {
    SpeechGate gate;
    std::deque<int16_t> prebuffer;   // ~1 sec of audio
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

        // === Buffers ===
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
            const size_t got = alsa.read(buf);
            if (got == 0) continue;

            // Stereo → mono + DC remove + gain
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
                                           frame480.data(), frame480.size());
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
			std::cout << "[Wake Word] Detected index " << keyword_index << std::endl;
			wake_detected = true;
			recording = true;
			clip_buffer.clear();
		    }

		    // Remove processed samples
		    porcupine_frame.erase(porcupine_frame.begin(),
					 porcupine_frame.begin() + 512);
		}

		// --- If recording, store the raw frames ---
		if (recording) {
		    clip_buffer.insert(clip_buffer.end(), frame480.begin(), frame480.end());
		}

		// --- End of recording condition ---
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

