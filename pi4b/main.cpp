#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
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

// Wake‑word detector (stub that “hears” every 15s)
void wake_word_thread() {
    while (!g_quit.load()) {
        std::this_thread::sleep_for(15s);
        push({Ev::WakeWord});
    }
}

// TTS (speaking) stub: when asked to speak, it posts TtsStarted then, 3s later, TtsFinished.
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
