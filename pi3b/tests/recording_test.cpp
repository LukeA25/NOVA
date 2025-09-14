#include <alsa/asoundlib.h>
#include <sndfile.h>
#include <iostream>
#include <vector>
#include <string>
#include <cstdlib>
#include <cstdint>
#include <climits>

int main(int argc, char* argv[]) {
    // Default settings
    const char* device = "hw:2,0";   // update to match `arecord -l`
    unsigned int rate = 48000;       // Hz
    int channels = 2;
    float gain = 1.0f;               // default 1.0 = no amplification
    int seconds = 5;

    // If argv[1] is given, treat it as gain
    if (argc > 1) {
        gain = std::stof(argv[1]);
    }

    std::cout << "Recording " << seconds 
              << " seconds at " << rate << " Hz, "
              << channels << " channels, gain=" << gain << "x\n";

    // Open ALSA capture device
    snd_pcm_t* pcm_handle;
    snd_pcm_hw_params_t* params;
    if (snd_pcm_open(&pcm_handle, device, SND_PCM_STREAM_CAPTURE, 0) < 0) {
        std::cerr << "Error opening PCM device " << device << "\n";
        return 1;
    }
    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(pcm_handle, params);
    snd_pcm_hw_params_set_access(pcm_handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(pcm_handle, params, SND_PCM_FORMAT_S32_LE);
    snd_pcm_hw_params_set_channels(pcm_handle, params, channels);
    snd_pcm_hw_params_set_rate_near(pcm_handle, params, &rate, nullptr);
    snd_pcm_hw_params(pcm_handle, params);
    snd_pcm_prepare(pcm_handle);

    int frames_per_buffer = 1024;
    std::vector<int32_t> buffer(frames_per_buffer * channels);
    std::vector<int32_t> all_samples;
    all_samples.reserve(rate * seconds * channels);

    // Capture loop
    int total_frames = 0;
    while (total_frames < rate * seconds) {
        snd_pcm_sframes_t got = snd_pcm_readi(pcm_handle, buffer.data(), frames_per_buffer);
        if (got < 0) {
            snd_pcm_recover(pcm_handle, got, 0);
            continue;
        }

        // Apply gain to each sample
        for (int i = 0; i < got * channels; i++) {
            int64_t v = static_cast<int64_t>(buffer[i]) * gain;
            if (v > INT32_MAX) v = INT32_MAX;
            if (v < INT32_MIN) v = INT32_MIN;
            buffer[i] = static_cast<int32_t>(v);
        }

        // Append to master buffer
        all_samples.insert(all_samples.end(), buffer.begin(), buffer.begin() + got * channels);

        total_frames += got;
    }

    snd_pcm_close(pcm_handle);

    // Save as WAV with libsndfile
    SF_INFO sfinfo;
    sfinfo.samplerate = rate;
    sfinfo.channels = channels;
    sfinfo.format = SF_FORMAT_WAV | SF_FORMAT_PCM_32;

    SNDFILE* out = sf_open("test.wav", SFM_WRITE, &sfinfo);
    if (!out) {
        std::cerr << "Error opening test.wav for writing\n";
        return 1;
    }

    sf_write_int(out, all_samples.data(), all_samples.size());
    sf_close(out);

    std::cout << "Saved test.wav with " << total_frames << " frames\n";
    return 0;
}
