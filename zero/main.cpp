// Copyright (c) 2025 Luke Anderson – MIT License

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <cstring>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <gpiod.h>

using namespace std::chrono_literals;

// ----------------------- Config -----------------------
namespace cfg {
    // -------- Wi-Fi / UDP --------
    inline const char* kUdpTargetIp = "10.20.0.195";
    constexpr uint16_t kUdpTxPort   = 5006; // Outgoing telemetry
    constexpr uint16_t kUdpRxPort   = 5005; // Incoming control

    // -------- RGB "eyes" control --------
    constexpr int kEyeRedPin   = 12;
    constexpr int kEyeBluePin  = 6;
    constexpr int kEyeGreenPin = 13;

    // -------- Rotary encoders (3 encoders, 2 pins each) --------
    constexpr int kEnc1A = 9;
    constexpr int kEnc1B = 11;
    constexpr int kEnc2A = 22;
    constexpr int kEnc2B = 10;
    constexpr int kEnc3A = 24;
    constexpr int kEnc3B = 27;

    // -------- Fuel gauge (MAX17043) status pins --------
    constexpr int kFuelGaugeAlertPin = 17;
    constexpr int kFuelGaugeSdaPin   = 0;
    constexpr int kFuelGaugeSclPin   = 1;

    // -------- Charger pins --------
    inline const char* kGpioChip    = "/dev/gpiochip0";
    constexpr int kChargePin        = 8;
    constexpr int kChargedPin       = 25;
}
// ------------------------------------------------------

// ------------------- State Machine Setup -------------------
enum class Ev { IdleTick, ScanTick, WakeWord, TtsStarted, TtsFinished, ChargeStarted, ChargeStopped };
struct Event { Ev id; };

enum class LedMode {
    Idle,        // Solid blue with occasional blink
    Charging,    // Blink orange
    Charged,     // Solid green
    Speaking     // Rapid blink blue
};
std::atomic<LedMode> g_led_mode{LedMode::Idle};

std::mutex g_q_m;
std::condition_variable g_q_cv;
std::queue<Event> g_q;

std::atomic<bool>  g_quit{false};

static void push(Event e) {
    std::lock_guard<std::mutex> lk(g_q_m);
    g_q.push(e);
    g_q_cv.notify_one();
}
static void on_sigint(int) {
    g_quit.store(true);
    g_q_cv.notify_all();
}
// ------------------------------------------------------

// ------------------- WiFi TX Function -------------------
void send_udp_msg(const char* msg) {
    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("[wifi_tx] socket");
        return;
    }

    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port   = htons(cfg::kUdpTxPort);
    dst.sin_addr.s_addr = inet_addr(cfg::kUdpTargetIp);

    ssize_t sent = sendto(sock, msg, strlen(msg), 0,
                          (sockaddr*)&dst, sizeof(dst));
    if (sent < 0) {
        perror("[wifi_tx] sendto");
    } else {
        std::cout << "[wifi_tx] Sent: " << msg << "\n";
    }

    ::close(sock);
}
// ------------------------------------------------------

// ------------------- Producers -------------------
void wifi_rx_thread() {
    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return; }

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

    std::string tmpPath = "/tmp/received.mp3";
    std::vector<char> buffer(64 * 1024);

    while (!g_quit.load()) {
        FILE* f = fopen(tmpPath.c_str(), "wb");
        if (!f) {
            perror("fopen");
            break;
        }

        while (!g_quit.load()) {
            ssize_t n = recv(sock, buffer.data(), buffer.size(), 0);
            if (n <= 0) continue;

            // Check for END signal
            if (n == 3 && std::string(buffer.data(), 3) == "END") {
                std::cout << "[WiFi] Received END signal\n";
                break; }

            std::cout << "[WiFi] Received " << n << " bytes\n";
            fwrite(buffer.data(), 1, n, f);
            fflush(f);
        }

        fclose(f);

        // Phase 2: Play file
        push(Event{Ev::TtsStarted});
        std::string cmd = "mpg123 -q " + tmpPath; // -f 65536 
        int ret = system(cmd.c_str());
        if (ret != 0) std::cerr << "Failed to play audio\n";
        push(Event{Ev::TtsFinished});
    }

    ::close(sock);
}

void gpio_thread() {
    gpiod_chip* chip = gpiod_chip_open(cfg::kGpioChip);
    if (!chip) {
        perror("gpiod_chip_open");
        return;
    }

    gpiod_line* charge = gpiod_chip_get_line(chip, cfg::kChargePin);
    gpiod_line* charged_line = gpiod_chip_get_line(chip, cfg::kChargedPin);

    gpiod_line_request_config config{ "nova-gpio", GPIOD_LINE_REQUEST_EVENT_BOTH_EDGES, 0 };
    gpiod_line_request(charge_line,  &config, 0);
 
    gpiod_line_request_config config{ "nova-gpio", GPIOD_LINE_REQUEST_EVENT_BOTH_EDGES, 0 };
    gpiod_line_request(charge_line,  &config, 0);
    gpiod_line_request(charged_line, &config, 0);

    while (!g_quit.load()) {
        gpiod_line_event ev;
-        if (gpiod_line_event_wait(charge_line, nullptr) > 0 &&
-            gpiod_line_event_read(charge_line, &ev) == 0) {
-            if (ev.event_type == GPIOD_LINE_EVENT_RISING_EDGE)
-                push(Event{Ev::ChargeStarted});
-            else
-                push(Event{Ev::ChargeStopped});
        }

        if (gpiod_line_event_wait(charged_line, nullptr) > 0 &&
            gpiod_line_event_read(charged_line, &ev) == 0) {
            // TODO: Maybe push a “charged” event here
        }
    }

    gpiod_chip_close(chip);
}

void led_thread() {
    gpiod_chip* chip = gpiod_chip_open(cfg::kGpioChip);
    if (!chip) { perror("gpiod_chip_open"); return; }

    auto open = [&](int pin) -> gpiod_line* {
        gpiod_line* line = gpiod_chip_get_line(chip, pin);
        gpiod_line_request_output(line, "led", 0);
        return line;
    };

    gpiod_line* red   = open(cfg::kEyeRedPin);
    gpiod_line* green = open(cfg::kEyeGreenPin);
    gpiod_line* blue  = open(cfg::kEyeBluePin);

    auto set_color = [&](bool r, bool g, bool b) {
        gpiod_line_set_value(red, r);
        gpiod_line_set_value(green, g);
        gpiod_line_set_value(blue, b);
    };

    int tick = 0;
    while (!g_quit.load()) {
        LedMode mode = g_led_mode.load();

        switch (mode) {
            case LedMode::Idle:
                set_color(0, 0, 1);
                if (tick % 40 == 0 || 1) set_color(0, 0, 0);
                break;
            case LedMode::Charging:
                set_color(0.5 * (tick % 10 < 5), 0.5 * (tick % 10 < 5), 0);
                break;
            case LedMode::Charged:
                set_color(0, 1, 0);
                break;
            case LedMode::Speaking:
                set_color(0, 0, (tick % 4 < 2));
                break;
        }

        std::this_thread::sleep_for(100ms);
        tick++;
    }

    gpiod_chip_close(chip);
}
// ------------------------------------------------------

// ------------------- Consumer -------------------
int main() {
    std::signal(SIGINT, on_sigint);

    std::thread(wifi_rx_thread).detach();
    std::thread(led_thread).detach();
    std::thread(gpio_thread).detach();

    while (!g_quit.load()) {
        Event ev;
        {
            std::unique_lock<std::mutex> lk(g_q_m);
            g_q_cv.wait(lk, []{ return !g_q.empty() || g_quit.load(); });
            if (g_quit.load()) break;
            ev = g_q.front(); g_q.pop();
        }

        switch (ev.id) {
            case Ev::ChargeStarted:
                std::cout << "[GPIO] Charge started\n";
                g_led_mode = LedMode::Charging;
                send_udp_msg(R"({"type":"charge","state":"start"})");
                break;
            case Ev::ChargeStopped:
                std::cout << "[GPIO] Charge stopped\n";
                g_led_mode = LedMode::Idle;
                send_udp_msg(R"({"type":"charge","state":"stop"})");
                break;
            case Ev::Charged:
                std::cout << "[GPIO] Charge stopped\n";
                g_led_mode = LedMode::Idle;
                break;
            case Ev::TtsStarted:
                g_led_mode = LedMode::Speaking;
                break;
            case Ev::TtsFinished:
                g_led_mode = LedMode::Idle;
                break;
            default:
                break;
        }
    }

    std::cout << "Exiting...\n";
    return 0;
}
