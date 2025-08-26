#pragma once

#ifndef __APPLE__
#include <alsa/asoundlib.h>
#endif

#include <vector>
#include <string>

class AlsaCapture {
public:
    AlsaCapture(const std::string& device,
                unsigned rate,
                unsigned channels,
                snd_pcm_format_t format = SND_PCM_FORMAT_S32_LE,
                snd_pcm_uframes_t frames_per_read = 256);

    ~AlsaCapture();

    // Blocking read: fills buffer with interleaved samples
    // returns number of frames actually read
    snd_pcm_sframes_t read(std::vector<int32_t>& buffer);

    unsigned channels() const { return m_channels; }
    unsigned rate() const { return m_rate; }

private:
    snd_pcm_t* m_pcm;
    unsigned m_rate;
    unsigned m_channels;
    snd_pcm_format_t m_format;
    snd_pcm_uframes_t m_frames_per_read;
};
