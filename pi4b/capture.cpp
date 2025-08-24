#include "capture.hpp"
#include <cstdio>
#include <stdexcept>

static void fail_and_throw(const char* where, int err) {
    std::fprintf(stderr, "%s: %s\n", where, snd_strerror(err));
    throw std::runtime_error("ALSA error");
}

AlsaCapture::AlsaCapture(std::string device,
                         unsigned rate,
                         unsigned channels,
                         snd_pcm_format_t format,
                         snd_pcm_uframes_t period_frames)
    : device_(std::move(device)),
      rate_(rate),
      channels_(channels),
      format_(format),
      period_frames_(period_frames) {}

AlsaCapture::~AlsaCapture() {
    stop();
}

void AlsaCapture::set_callback(Callback cb) {
    cb_ = std::move(cb);
}

bool AlsaCapture::start() {
    if (running_.load()) return true;
    if (!configure_()) return false;

    // Allocate buffer for one period
    buf_.resize(static_cast<size_t>(period_frames_) * channels_);

    running_ = true;
    thread_ = std::thread(&AlsaCapture::run_, this);
    return true;
}

void AlsaCapture::stop() {
    if (!running_.exchange(false)) return;
    if (thread_.joinable()) thread_.join();

    if (pcm_) {
        snd_pcm_close(pcm_);
        pcm_ = nullptr;
    }
    buf_.clear();
}

bool AlsaCapture::configure_() {
    // Open PCM capture device
    int err = snd_pcm_open(&pcm_, device_.c_str(), SND_PCM_STREAM_CAPTURE, 0);
    if (err < 0) { std::fprintf(stderr, "snd_pcm_open: %s\n", snd_strerror(err)); return false; }

    snd_pcm_hw_params_t* hp = nullptr;
    snd_pcm_hw_params_alloca(&hp);
    snd_pcm_hw_params_any(pcm_, hp);

    if ((err = snd_pcm_hw_params_set_access(pcm_, hp, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
        std::fprintf(stderr, "set_access: %s\n", snd_strerror(err)); return false;
    }
    if ((err = snd_pcm_hw_params_set_format(pcm_, hp, format_)) < 0) {
        std::fprintf(stderr, "set_format: %s\n", snd_strerror(err)); return false;
    }
    if ((err = snd_pcm_hw_params_set_rate(pcm_, hp, rate_, 0)) < 0) {
        std::fprintf(stderr, "set_rate: %s\n", snd_strerror(err)); return false;
    }
    if ((err = snd_pcm_hw_params_set_channels(pcm_, hp, channels_)) < 0) {
        std::fprintf(stderr, "set_channels: %s\n", snd_strerror(err)); return false;
    }
    snd_pcm_uframes_t period = period_frames_;
    if ((err = snd_pcm_hw_params_set_period_size_near(pcm_, hp, &period, 0)) < 0) {
        std::fprintf(stderr, "set_period: %s\n", snd_strerror(err)); return false;
    }
    period_frames_ = period;

    if ((err = snd_pcm_hw_params(pcm_, hp)) < 0) {
        std::fprintf(stderr, "hw_params: %s\n", snd_strerror(err)); return false;
    }
    return true;
}

void AlsaCapture::run_() {
    // Read loop
    while (running_.load()) {
        snd_pcm_sframes_t got = snd_pcm_readi(pcm_, buf_.data(), period_frames_);
        if (got == -EPIPE) {
            // Overrun — prepare and continue
            snd_pcm_prepare(pcm_);
            continue;
        }
        if (got < 0) {
            std::fprintf(stderr, "readi: %s\n", snd_strerror((int)got));
            continue;
        }

        if (cb_) {
            cb_(buf_.data(), static_cast<size_t>(got), channels_);
        }
    }
}
