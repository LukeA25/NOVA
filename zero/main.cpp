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

#ifndef __APPLE__
  #include <gpiod.h>
#endif

using namespace std::chrono_literals;

// ----------------------- Config -----------------------
namespace cfg {
    // -------- Wi-Fi / UDP --------
    inline const char* kUdpTargetIp = "10.20.0.195";
    constexpr uint16_t kUdpTxPort   = 5005; // Outgoing telemetry
    constexpr uint16_t kUdpRxPort   = 5006; // Incoming control

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

void wifi_tx_thread() {
    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return; }

    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port   = htons(cfg::kUdpTxPort);
    dst.sin_addr.s_addr = inet_addr(cfg::kUdpTargetIp);

    while (!g_quit.load()) {
        // TODO: Package encoder data and send with sendto()
        std::this_thread::sleep_for(20ms);
    }

    ::close(sock);
}

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

    // Allow large enough buffer for small MP3s
    std::vector<char> buffer(64 * 1024);  // 64 KB
    while (!g_quit.load()) {
        ssize_t n = recv(sock, buffer.data(), buffer.size(), 0);
        if (n > 0) {
            std::cout << "[WiFi] Received " << n << " bytes\n";

            // Write to temp file
            std::string tmpPath = "/tmp/received.mp3";
            FILE* f = fopen(tmpPath.c_str(), "wb");
            if (!f) {
                perror("fopen");
                continue;
            }
            fwrite(buffer.data(), 1, n, f);
            fclose(f);

            // Play it using mpg123
            std::string cmd = "mpg123 -q " + tmpPath;
            int ret = system(cmd.c_str());
            if (ret != 0)
                std::cerr << "Failed to play audio\n";
        }
    }

    ::close(sock);
}

void gpio_thread() {
#ifndef __APPLE__
    gpiod_chip* chip = gpiod_chip_open(cfg::kGpioChip);
    if (!chip) { perror("gpiod_chip_open"); return; }

    gpiod_line* charge_line  = gpiod_chip_get_line(chip, cfg::kChargePin);
    gpiod_line* charged_line = gpiod_chip_get_line(chip, cfg::kChargedPin);

    gpiod_line_request_config config{ "nova-gpio", GPIOD_LINE_REQUEST_EVENT_BOTH_EDGES, 0 };
    gpiod_line_request(charge_line,  &config, 0);
    gpiod_line_request(charged_line, &config, 0);

    while (!g_quit.load()) {
        gpiod_line_event ev;
        if (gpiod_line_event_wait(charge_line, nullptr) > 0 &&
            gpiod_line_event_read(charge_line, &ev) == 0) {
            if (ev.event_type == GPIOD_LINE_EVENT_RISING_EDGE)
                push(Event{Ev::ChargeStarted});
            else
                push(Event{Ev::ChargeStopped});
        }
        if (gpiod_line_event_wait(charged_line, nullptr) > 0 &&
            gpiod_line_event_read(charged_line, &ev) == 0) {
            // TODO: Maybe push a “charged” event here
        }
    }

    gpiod_chip_close(chip);
#endif
}

// ------------------- Consumer -------------------

int main() {
    std::signal(SIGINT, on_sigint);

    std::thread(wifi_tx_thread).detach();
    std::thread(wifi_rx_thread).detach();
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
            case Ev::ChargeStarted: std::cout << "[GPIO] Charge started\n"; break;
            case Ev::ChargeStopped: std::cout << "[GPIO] Charge stopped\n"; break;
            default: break;
        }
    }

    std::cout << "Exiting...\n";
    return 0;
}
