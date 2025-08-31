#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include "nova/audio/AlsaCapture.hpp"
#include "nova/audio/doa_gcc_phat.hpp"

#ifndef __APPLE__
#include <alsa/asoundlib.h>
#endif

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

// Wake‑word detector (stub that “hears” every 15s)
void wake_word_thread() {
    // Adjust device string to your card if needed (check with `arecord -l`)
    AlsaCapture cap("hw:1,0", 16000, 2, SND_PCM_FORMAT_S32_LE, 256);

    constexpr float Fs = 16000.0f;
    constexpr float MIC_SPACING_M = 0.189f; // your 18.9 cm
    const int LEVEL_SHIFT = 8;              // downshift to keep sums sane
    const int64_t LEVEL_THRESH = 5e6;       // crude energy gate

    std::vector<int32_t> buf;
    uint64_t last_print_ms = 0;

    while (!g_quit.load()) {
        snd_pcm_sframes_t got = cap.read(buf);  // got = frames read
        if (got <= 0) continue;

        // crude level (mono sum of abs(left))
        int64_t sumAbs = 0;
        for (snd_pcm_sframes_t i = 0; i < got; ++i) {
            sumAbs += std::llabs((long long)buf[2*i + 0] >> LEVEL_SHIFT);
        }

        // Estimate direction once per buffer (or throttle prints)
        float deg = 0.0f;
        float conf = nova::audio::estimate_direction(buf.data(), (size_t)got, Fs, MIC_SPACING_M, &deg);

        // Throttle console spam to ~10 Hz
        uint64_t now_ms = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now().time_since_epoch()).count();
        if (now_ms - last_print_ms > 100) {
            std::cout << "[mic] level=" << sumAbs
                      << "  doa_deg=" << deg
                      << "  conf=" << conf << "\n";
            last_print_ms = now_ms;
        }

        // Super-simple “wake” gate: loud enough AND somewhat confident
        if (sumAbs > LEVEL_THRESH && conf > 0.1f) {
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
