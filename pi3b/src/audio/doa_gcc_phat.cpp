// src/audio/doa_gcc_phat.cpp
//
// Implementation of GCC-PHAT direction of arrival (DoA) estimation
// for NOVA project audio module.
//
// Author: Luke Anderson
// License: MIT

#include "nova/audio/doa_gcc_phat.hpp"

#include <vector>
#include <cmath>
#include <algorithm>
#include "kiss_fft.h"

namespace nova::audio {

static inline size_t next_pow2(size_t n) {
    size_t p = 1;
    while (p < n) p <<= 1;
    return p;
}

static inline float hann(size_t i, size_t N) {
    // Hann window to reduce spectral leakage
    constexpr float kPi = 3.14159265358979323846f;
    return 0.5f - 0.5f * std::cos(2.0f * kPi * float(i) / float(N - 1));
}

// ------------------ FUNCTION IMPLEMENTATION ------------------

float estimate_direction(const int32_t* buf,
                         std::size_t frames,
                         float Fs,
                         float mic_spacing_m,
                         float* out_deg)
{
    if (!buf || frames == 0 || Fs <= 0 || mic_spacing_m <= 0 || !out_deg) {
        if (out_deg) *out_deg = 0.0f;
        return 0.0f;
    }

    // 1) Deinterleave and scale 24-bit-in-32 format to float and window
    std::vector<float> L(frames), R(frames);
    for (size_t i = 0; i < frames; ++i) {
        // 24-bit signed PCM packed in int32_t. Scale to ~[-1,1].
        float w = hann(i, frames);
        L[i] = (buf[2*i + 0] / 8388608.0f) * w;  // 2^23 = 8388608
        R[i] = (buf[2*i + 1] / 8388608.0f) * w;
    }

    // 2) Choose FFT size
    size_t Nfft = next_pow2(frames * 2);
    if (Nfft < 64) Nfft = 64;

	// 3) Pack inputs for KissFFT
    std::vector<kiss_fft_cpx> X(Nfft), Y(Nfft), G(Nfft), c(Nfft);
    for (size_t i = 0; i < Nfft; ++i) {
        X[i].r = (i < L.size()) ? L[i] : 0.0f;  X[i].i = 0.0f;
        Y[i].r = (i < R.size()) ? R[i] : 0.0f;  Y[i].i = 0.0f;
    }

    // 4) Forward FFTs
    kiss_fft_cfg fwd = kiss_fft_alloc(int(Nfft), 0, nullptr, nullptr);
    kiss_fft_cfg inv = kiss_fft_alloc(int(Nfft), 1, nullptr, nullptr);
    // if (!fwd || !inv) {
    //     if (out_deg) *out_deg = 0.0f;
    //     if (fwd) free(fwd);
    //     if (inv) free(inv);
    //     return 0.0f;------------------------------------------------------------
    // }
    //
    // kiss_fft_cpx XL, YL;
    kiss_fft(fwd, X.data(), X.data()); // in-place FFT of L
    kiss_fft(fwd, Y.data(), Y.data()); // in-place FFT of R

    // 5) GCC-PHAT weighting (frequency domain)
    for (size_t k = 0; k < Nfft; ++k) {
        const float xr = X[k].r, xi = X[k].i;
        const float yr = Y[k].r, yi = Y[k].i;
        // X * conj(Y) = (xr + j xi) * (yr - j yi)
        float gr = xr*yr + xi*yi;
        float gi = xi*yr - xr*yi;
        float mag = std::sqrt(gr*gr + gi*gi) + 1e-12f; // avoid /0
        G[k].r = gr / mag;
        G[k].i = gi / mag;
    }

    // 6) Inverse FFT -> cross-correlation sequence
    kiss_fft(inv, G.data(), c.data());

    for (size_t n = 0; n < Nfft; ++n) {
        c[n].r /= float(Nfft);
        c[n].i /= float(Nfft);
    }

    // 7) Search for plausible lags
    const int maxLag = int(std::floor((mic_spacing_m / 343.0f) * Fs));
    int bestLag = 0;
    float bestVal = -1e30f;

    const int Nfft_i = int(Nfft);
    for (int lag = -maxLag; lag <= maxLag; ++lag) {
        int idx = lag;
        idx %= Nfft_i;
        if (idx < 0) idx += Nfft_i;
        float v = c[size_t(idx)].r; // real part
        if (v > bestVal) { bestVal = v; bestLag = lag; }
    }

    // 8) Convert lag -> angle
    const float tau = bestLag / Fs;
    float s = (343.0f * tau) / mic_spacing_m;   // sin(theta)
    if (s >  1.0f) s =  1.0f;
    if (s < -1.0f) s = -1.0f;
    *out_deg = std::asin(s) * 180.0f / float(M_PI);

    free(fwd);
    free(inv);

    return std::max(0.0f, std::min(bestVal, 1.0f));
}

} // namespace nova::audio
