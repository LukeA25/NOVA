#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

#include <alsa/asoundlib.h>
#include <sndfile.h>

int main() {
    const char* device = "hw:2,0";   // adjust with `arecord -l`
    const unsigned rate = 48000;
    const unsigned channels = 2;
    const snd_pcm_format_t fmt = SND_PCM_FORMAT_S32_LE;
    const snd_pcm_uframes_t frames_per_period = 256;

    // Open ALSA capture
    snd_pcm_t* handle;
    if (snd_pcm_open(&handle, device, SND_PCM_STREAM_CAPTURE, 0) < 0) {
        std::cerr << "Failed to open ALSA device\n";
        return 1;
    }

    // Set parameters
    snd_pcm_set_params(handle, fmt,
                       SND_PCM_ACCESS_RW_INTERLEAVED,
                       channels, rate, 1, 500000); // 0.5s latency

    // Prepare buffer for 5 seconds of audio
    size_t total_frames = rate * 5; // 5 seconds
    std::vector<int32_t> buf(total_frames * channels);

    // Capture
    size_t captured = 0;
    while (captured < total_frames) {
        snd_pcm_sframes_t got = snd_pcm_readi(handle, buf.data() + captured * channels,
                                              total_frames - captured);
        if (got < 0) {
            snd_pcm_recover(handle, (int)got, 0);
            continue;
        }
        captured += got;
    }

    snd_pcm_close(handle);

    // Save to WAV in build/
    SF_INFO sfinfo{};
    sfinfo.channels   = channels;
    sfinfo.samplerate = rate;
    sfinfo.format     = SF_FORMAT_WAV | SF_FORMAT_PCM_32;

    SNDFILE* out = sf_open("test.wav", SFM_WRITE, &sfinfo);
    if (!out) {
        std::cerr << "Failed to open output file\n";
        return 1;
    }

    sf_write_int(out, buf.data(), buf.size());
    sf_close(out);

    std::cout << "Saved test.wav (" << captured << " frames)\n";
    return 0;
}
