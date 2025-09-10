#pragma once
#ifndef NOVA_AUDIO_DOA_GCC_PHAT_HPP
#define NOVA_AUDIO_DOA_GCC_PHAT_HPP

#include <cstdint>
#include <cstddef>

namespace nova::audio {

/**
 * @brief Estimate 2-mic DoA using GCC-PHAT.
 *
 * @param interleaved_s32  Interleaved S32_LE PCM: L,R,L,R,… length = frames*2.
 * @param frames           Stereo frames in the buffer.
 * @param sample_rate_hz   Sample rate (e.g., 16000).
 * @param mic_spacing_m    Mic spacing in meters (e.g., 0.189).
 * @param out_deg          [out] Angle in degrees (left −, right +).
 * @return                 Confidence in ~[0,1]. Returns 0 on invalid input.
 */
float estimate_direction(const int32_t* interleaved_s32,
                         std::size_t    frames,
                         float          sample_rate_hz,
                         float          mic_spacing_m,
                         float*         out_deg);

} // namespace nova::audio

#endif // NOVA_AUDIO_DOA_GCC_PHAT_HPP
