#pragma once

#include <alsa/asoundlib.h>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <atomic>
#include <vector>

class AlsaCapture {
public:
    // interleaved points to frames*channels samples of S32_LE
    using Callback = std::function<void(const int32_t* interleaved,
                                        size_t frames,
                                        unsigned channels)>;

    // device like "hw:1,0"; rate in Hz; channels 1 or 2; format usually SND_PCM_FORMAT_S32_LE
    explicit AlsaCapture(std::string device = "hw:1,0",
                         unsigned rate = 16000,
                         unsigned channels = 2,
                         snd_pcm_format_t format = SND_PCM_FORMAT_S32_LE,
                         snd_pcm_uframes_t period_frames = 256);

    ~AlsaCapture();

    // Set or replace the callback that receives audio buffers
    void set_callback(Callback cb);

    // Start/stop the background capture thread
    bool start();
    void stop();
    bool is_running() const { return running_.load(); }

    // Introspection
    unsigned rate() const { return rate_; }
    unsigned channels() const { return channels_; }
    snd_pcm_format_t format() const { return format_; }
    snd_pcm_uframes_t period_frames() const { return period_frames_; }
    const std::string& device() const { return device_; }

private:
    bool configure_();   // Open + configure ALSA device
    void run_();         // Thread entry

    std::string device_;
    unsigned rate_;
    unsigned channels_;
    snd_pcm_format_t format_;
    snd_pcm_uframes_t period_frames_;

    snd_pcm_t* pcm_ = nullptr;
    std::vector<int32_t> buf_;   // interleaved buffer

    std::thread thread_;
    std::atomic<bool> running_{false};
    Callback cb_ = nullptr;
};
