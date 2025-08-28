#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include "capture.hpp"
#include <alsa/asoundlib.h>

using namespace std::chrono_literals;

enum class State { CHARGING, IDLE, LISTENING, SPEAKING };

enum class Ev {
  IdleTick,        // “time to do an idle animation”
  ScanTick,        // “time to do a visual scan”
  WakeWord,        // wake word detected
  TtsStarted,      // speaking began
  TtsFinished      // speaking ended
};

struct Event { Ev id; };

std::mutex g_m;
std::condition_variable g_cv;
std::queue<Event> g_q;
std::atomic<bool> g_quit{false};

void push(Event e) {
    std::lock_guard<std::mutex> lk(g_m);
    g_q.push(e);
    g_cv.notify_one();
}

/* ---------- PRODUCERS ---------- */

// 30s idle timer (fires only while in IDLE; we just let the state machine ignore it otherwise)
void idle_timer() {
    while (!g_quit.load()) { std::this_thread::sleep_for(30s); push({Ev::IdleTick}); }
}

// 5m scan timer
void scan_timer() {
    while (!g_quit.load()) { std::this_thread::sleep_for(5min); push({Ev::ScanTick}); }
}

// Estimate direction of voice input
float estimate_direction(const int32_t* buf, size_t frames, float Fs, float mic_spacing_m,
                        float* out_deg)
{
    std::vector<float> L(frames), R(frames);
    for (size_t i = 0; i < frames; ++i) {
        L[i] = buf[2*i + 0] * (1.0f / 8388608.0f);
        R[i] = buf[2*i + 1] * (1.0f / 8388608.0f);
    }

    // 2) FFT(L), FFT(R)  -> use any FFT lib (KissFFT, FFTW)
	std::vector<std::complex<float>> LF, RF;
	fft(L, LF); fft(R, RF);

    // 3) GCC-PHAT
    for (k) G[k] = (LF[k]*conj(RF[k])) / (abs(LF[k]*conj(RF[k])) + 1e-12);

    // 4) IFFT(G) -> c[lag]
    ifft(G, c);

    int maxLag = (int)std::floor((mic_spacing_m / 343.0f) * Fs);
    int bestLag = 0; float bestVal = -1e9f;
    for (int lag = -maxLag; lag <= maxLag; ++lag) {
        float v = c_limited[lag]; // from IFFT result, shifted to [-N/2, +N/2]
        if (v > bestVal) { bestVal = v; bestLag = lag; }
    }

    // 6) angle
    float tau = bestLag / Fs;
    float s = (343.0f * tau) / mic_spacing_m;
    if (s > 1.f) s = 1.f; else if (s < -1.f) s = -1.f;
    *out_deg = std::asin(s) * 180.0f / float(M_PI);
}

// Wake‑word detector (stub that “hears” every 15s)
void wake_word_thread() {
	AlsaCapture cap("hw:1,0", 16000, 2, SND_PCM_FORMAT_S32_LE, 256);

    std::vector<int32_t> buf;
    while (!g_quit.load()) {
        snd_pcm_sframes_t got = cap.read(buf);
        if (got <= 0) continue;

        // crude level detector
        int64_t sumAbs = 0;
        for (snd_pcm_sframes_t i = 0; i < got; ++i) {
            sumAbs += std::llabs((long long)buf[2*i] >> 8);
        }

        std::cout << "[wake] level=" << sumAbs << "\n";

        if (sumAbs > 5e6) {
            push({Ev::WakeWord});
        }
    }
}

void start_tts_async(const std::string& text) {
    std::thread([text]{
        push({Ev::TtsStarted});
        std::this_thread::sleep_for(3s);
        push({Ev::TtsFinished});
    }).detach();
}

/* ---------- STATE MACHINE (CONSUMER) ---------- */

int main() {
    std::thread(idle_timer).detach();
    std::thread(scan_timer).detach();
    std::thread(wake_word_thread).detach();

    State state = State::IDLE;

    auto do_idle_animation = []{
        std::cout << "[idle] animation\n";
        // kick a brief animation; return quickly so we stay responsive
    };

    auto do_visual_scan = []{
        std::cout << "[idle] visual scan\n";
        // fire off a non-blocking scan or a short blocking one
    };

    auto begin_listening = []{
        std::cout << "[listening] start\n";
        // start ASR (mic capture); when you get a request to speak,
        // call start_tts_async("..."); then you’ll get TtsStarted/TtsFinished
    };

    while (!g_quit.load()) {
        Event ev;
        {
            std::unique_lock<std::mutex> lk(g_m);
            g_cv.wait(lk, []{ return !g_q.empty(); });
            ev = g_q.front(); g_q.pop();
        }

        switch (state) {
            case State::CHARGING:
                
                break;

            case State::IDLE:
                if (ev.id == Ev::WakeWord) {
                    state = State::LISTENING;
                    begin_listening();
                } else if (ev.id == Ev::IdleTick) {
                    do_idle_animation();
                } else if (ev.id == Ev::ScanTick) {
                    do_visual_scan();
                } else if (ev.id == Ev::TtsStarted) {
                    // If you trigger speech from IDLE (e.g., a scheduled prompt)
                    state = State::SPEAKING;
                }
                break;

            case State::LISTENING:
                if (ev.id == Ev::TtsStarted) {
                    state = State::SPEAKING;
                }
                // (Other listening events would be handled here)
                break;

            case State::SPEAKING:
                // IMPORTANT RULE: speaking is *not* interrupted by wake‑word.
                if (ev.id == Ev::TtsFinished) {
                    state = State::IDLE;
                }
                // Ignore IdleTick/ScanTick/WakeWord while speaking.
                break;
        }
    }
}
