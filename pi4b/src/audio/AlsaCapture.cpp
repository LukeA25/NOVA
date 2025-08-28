#include "capture.hpp"
#include <cstdio>

static void fail(const char* msg, int err) {
    std::fprintf(stderr, "%s: %s\n", msg, snd_strerror(err));
    throw std::runtime_error(msg);
}

AlsaCapture::AlsaCapture(const std::string& device,
                         unsigned rate,
                         unsigned channels,
                         snd_pcm_format_t format,
                         snd_pcm_uframes_t frames_per_read)
    : m_pcm(nullptr),
      m_rate(rate),
      m_channels(channels),
      m_format(format),
      m_frames_per_read(frames_per_read)
{
    if (int err = snd_pcm_open(&m_pcm, device.c_str(), SND_PCM_STREAM_CAPTURE, 0); err < 0)
        fail("snd_pcm_open", err);

    snd_pcm_hw_params_t* hp;
    snd_pcm_hw_params_alloca(&hp);
    snd_pcm_hw_params_any(m_pcm, hp);

    if (int err = snd_pcm_hw_params_set_access(m_pcm, hp, SND_PCM_ACCESS_RW_INTERLEAVED); err < 0) fail("set_access", err);
    if (int err = snd_pcm_hw_params_set_format(m_pcm, hp, m_format); err < 0) fail("set_format", err);
    if (int err = snd_pcm_hw_params_set_rate(m_pcm, hp, m_rate, 0); err < 0) fail("set_rate", err);
    if (int err = snd_pcm_hw_params_set_channels(m_pcm, hp, m_channels); err < 0) fail("set_channels", err);

    snd_pcm_uframes_t per = m_frames_per_read;
    if (int err = snd_pcm_hw_params_set_period_size_near(m_pcm, hp, &per, 0); err < 0) fail("set_period", err);

    if (int err = snd_pcm_hw_params(m_pcm, hp); err < 0) fail("hw_params", err);
}

AlsaCapture::~AlsaCapture() {
    if (m_pcm) snd_pcm_close(m_pcm);
}

snd_pcm_sframes_t AlsaCapture::read(std::vector<int32_t>& buffer) {
    buffer.resize(m_frames_per_read * m_channels);
    snd_pcm_sframes_t got = snd_pcm_readi(m_pcm, buffer.data(), m_frames_per_read);

    if (got == -EPIPE) {
        snd_pcm_prepare(m_pcm); // recover from overrun
        return 0;
    }
    if (got < 0) {
        fail("readi", (int)got);
    }
    return got;
}
