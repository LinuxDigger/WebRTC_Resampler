/*
 *  Copyright (c) 2011 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef _WEBRTC_RESAMPLER_H_
#define _WEBRTC_RESAMPLER_H_
#ifndef  nullptr
#define  nullptr NULL
#endif

#include <stdio.h>
#include <stddef.h>

// Processor architecture detection.  For more info on what's defined, see:
//   http://msdn.microsoft.com/en-us/library/b0084kay.aspx
//   http://www.agner.org/optimize/calling_conventions.pdf
//   or with gcc, run: "echo | gcc -E -dM -"
#if defined(_M_X64) || defined(__x86_64__)
#define WEBRTC_ARCH_X86_FAMILY
#define WEBRTC_ARCH_64_BITS
#define WEBRTC_ARCH_LITTLE_ENDIAN
#elif defined(__aarch64__)
#define WEBRTC_ARCH_ARM_FAMILY
#define WEBRTC_ARCH_64_BITS
#define WEBRTC_ARCH_LITTLE_ENDIAN
#elif defined(_M_IX86) || defined(__i386__)
#define WEBRTC_ARCH_X86_FAMILY
#define WEBRTC_ARCH_X86
#define WEBRTC_ARCH_32_BITS
#define WEBRTC_ARCH_LITTLE_ENDIAN
#elif defined(__ARMEL__)
#define WEBRTC_ARCH_ARM_FAMILY
#define WEBRTC_ARCH_32_BITS
#define WEBRTC_ARCH_LITTLE_ENDIAN
#elif defined(__MIPSEL__)
#define WEBRTC_ARCH_MIPS_FAMILY
#if defined(__LP64__)
#define WEBRTC_ARCH_64_BITS
#else
#define WEBRTC_ARCH_32_BITS
#endif
#define WEBRTC_ARCH_LITTLE_ENDIAN
#elif defined(__pnacl__)
#define WEBRTC_ARCH_32_BITS
#define WEBRTC_ARCH_LITTLE_ENDIAN
#else
#error Please add support for your architecture in typedefs.h
#endif

#if !(defined(WEBRTC_ARCH_LITTLE_ENDIAN) ^ defined(WEBRTC_ARCH_BIG_ENDIAN))
#error Define either WEBRTC_ARCH_LITTLE_ENDIAN or WEBRTC_ARCH_BIG_ENDIAN
#endif

// TODO(zhongwei.yao): WEBRTC_CPU_DETECTION is only used in one place; we should
// probably just remove it.
#if (defined(WEBRTC_ARCH_X86_FAMILY) && !defined(__SSE2__))
#define WEBRTC_CPU_DETECTION
#endif

#include <stdint.h>

// Annotate a function indicating the caller must examine the return value.
// Use like:
//   int foo() RTC_WARN_UNUSED_RESULT;
// To explicitly ignore a result, cast to void.
// TODO(kwiberg): Remove when we can use [[nodiscard]] from C++17.
#if defined(__clang__)
#define RTC_WARN_UNUSED_RESULT __attribute__((__warn_unused_result__))
#elif defined(__GNUC__)
// gcc has a __warn_unused_result__ attribute, but you can't quiet it by
// casting to void, so we don't use it.
#define RTC_WARN_UNUSED_RESULT
#else
#define RTC_WARN_UNUSED_RESULT
#endif

// Put after a variable that might not be used, to prevent compiler warnings:
//   int result ATTRIBUTE_UNUSED = DoSomething();
//   assert(result == 17);
// Deprecated since it only works with GCC & clang. See RTC_UNUSED below.
// TODO(terelius): Remove.
#ifndef ATTRIBUTE_UNUSED
#if defined(__GNUC__) || defined(__clang__)
#define ATTRIBUTE_UNUSED __attribute__ ((__unused__))
#else
#define ATTRIBUTE_UNUSED
#endif
#endif

// Macro to be used for switch-case fallthrough (required for enabling
// -Wimplicit-fallthrough warning on Clang).
#ifndef FALLTHROUGH
#if defined(__clang__)
#define FALLTHROUGH() [[clang::fallthrough]]
#else
#define FALLTHROUGH() do { } while (0)
#endif
#endif

#ifndef NO_RETURN
// Annotate a function that will not return control flow to the caller.
#if defined(_MSC_VER)
#define NO_RETURN __declspec(noreturn)
#elif defined(__GNUC__)
#define NO_RETURN __attribute__ ((__noreturn__))
#else
#define NO_RETURN
#endif
#endif

// Prevent the compiler from warning about an unused variable. For example:
//   int result = DoSomething();
//   assert(result == 17);
//   RTC_UNUSED(result);
// Note: In most cases it is better to remove the unused variable rather than
// suppressing the compiler warning.
#ifndef RTC_UNUSED
#define RTC_UNUSED(x) static_cast<void>(x)
#endif  // RTC_UNUSED


/*******************************************************************
 * resample_by_2_fast.c
 * Functions for internal use in the other resample functions
 ******************************************************************/
void WebRtcSpl_DownBy2IntToShort(int32_t *in, int32_t len, int16_t *out,
                                 int32_t *state);

void WebRtcSpl_UpBy2ShortToInt(const int16_t *in, int32_t len,
                               int32_t *out, int32_t *state);

void WebRtcSpl_UpBy2IntToShort(const int32_t *in, int32_t len,
                               int16_t *out, int32_t *state);

void WebRtcSpl_LPBy2ShortToInt(const int16_t *in, int32_t len,
                               int32_t *out, int32_t *state);


#include <string.h>

// C + the 32 most significant bits of A * B
#define WEBRTC_SPL_SCALEDIFF32(A, B, C) \
    ((C) + ((B) >> 16) * (A) + (((uint32_t)((B) & 0x0000FFFF) * (A)) >> 16))


#ifdef __cplusplus
extern "C" {
#endif

/************************************************************
 *
 * RESAMPLING FUNCTIONS AND THEIR STRUCTS ARE DEFINED BELOW
 *
 ************************************************************/

/*******************************************************************
 * resample.c
 *
 * Includes the following resampling combinations
 * 22 kHz -> 16 kHz
 * 16 kHz -> 22 kHz
 * 22 kHz ->  8 kHz
 *  8 kHz -> 22 kHz
 *
 ******************************************************************/

// state structure for 22 -> 16 resampler
typedef struct {
    int32_t S_22_44[8];
    int32_t S_44_32[8];
    int32_t S_32_16[8];
} WebRtcSpl_State22khzTo16khz;

void WebRtcSpl_Resample22khzTo16khz(const int16_t *in,
                                    int16_t *out,
                                    WebRtcSpl_State22khzTo16khz *state,
                                    int32_t *tmpmem);

void WebRtcSpl_ResetResample22khzTo16khz(WebRtcSpl_State22khzTo16khz *state);

// state structure for 16 -> 22 resampler
typedef struct {
    int32_t S_16_32[8];
    int32_t S_32_22[8];
} WebRtcSpl_State16khzTo22khz;

void WebRtcSpl_Resample16khzTo22khz(const int16_t *in,
                                    int16_t *out,
                                    WebRtcSpl_State16khzTo22khz *state,
                                    int32_t *tmpmem);

void WebRtcSpl_ResetResample16khzTo22khz(WebRtcSpl_State16khzTo22khz *state);

// state structure for 22 -> 8 resampler
typedef struct {
    int32_t S_22_22[16];
    int32_t S_22_16[8];
    int32_t S_16_8[8];
} WebRtcSpl_State22khzTo8khz;

void WebRtcSpl_Resample22khzTo8khz(const int16_t *in, int16_t *out,
                                   WebRtcSpl_State22khzTo8khz *state,
                                   int32_t *tmpmem);

void WebRtcSpl_ResetResample22khzTo8khz(WebRtcSpl_State22khzTo8khz *state);

// state structure for 8 -> 22 resampler
typedef struct {
    int32_t S_8_16[8];
    int32_t S_16_11[8];
    int32_t S_11_22[8];
} WebRtcSpl_State8khzTo22khz;

void WebRtcSpl_Resample8khzTo22khz(const int16_t *in, int16_t *out,
                                   WebRtcSpl_State8khzTo22khz *state,
                                   int32_t *tmpmem);

void WebRtcSpl_ResetResample8khzTo22khz(WebRtcSpl_State8khzTo22khz *state);

/*******************************************************************
 * resample_fractional.c
 * Functions for internal use in the other resample functions
 *
 * Includes the following resampling combinations
 * 48 kHz -> 32 kHz
 * 32 kHz -> 24 kHz
 * 44 kHz -> 32 kHz
 *
 ******************************************************************/

void WebRtcSpl_Resample48khzTo32khz(const int32_t *In, int32_t *Out, size_t K);

void WebRtcSpl_Resample32khzTo24khz(const int32_t *In, int32_t *Out, size_t K);

void WebRtcSpl_Resample44khzTo32khz(const int32_t *In, int32_t *Out, size_t K);

/*******************************************************************
 * resample_48khz.c
 *
 * Includes the following resampling combinations
 * 48 kHz -> 16 kHz
 * 16 kHz -> 48 kHz
 * 48 kHz ->  8 kHz
 *  8 kHz -> 48 kHz
 *
 ******************************************************************/

typedef struct {
    int32_t S_48_48[16];
    int32_t S_48_32[8];
    int32_t S_32_16[8];
} WebRtcSpl_State48khzTo16khz;

void WebRtcSpl_Resample48khzTo16khz(const int16_t *in, int16_t *out,
                                    WebRtcSpl_State48khzTo16khz *state,
                                    int32_t *tmpmem);

void WebRtcSpl_ResetResample48khzTo16khz(WebRtcSpl_State48khzTo16khz *state);

typedef struct {
    int32_t S_16_32[8];
    int32_t S_32_24[8];
    int32_t S_24_48[8];
} WebRtcSpl_State16khzTo48khz;

void WebRtcSpl_Resample16khzTo48khz(const int16_t *in, int16_t *out,
                                    WebRtcSpl_State16khzTo48khz *state,
                                    int32_t *tmpmem);

void WebRtcSpl_ResetResample16khzTo48khz(WebRtcSpl_State16khzTo48khz *state);

/*******************************************************************
 * resample_by_2.c
 *
 * Includes down and up sampling by a factor of two.
 *
 ******************************************************************/

void WebRtcSpl_DownsampleBy2(const int16_t *in, size_t len,
                             int16_t *out, int32_t *filtState);

void WebRtcSpl_UpsampleBy2(const int16_t *in, size_t len,
                           int16_t *out, int32_t *filtState);

#ifdef __cplusplus
}
#endif  // __cplusplus

// All methods return 0 on success and -1 on failure.
class Resampler {
public:
    Resampler();

    Resampler(size_t inFreq, size_t outFreq, size_t num_channels);

    ~Resampler();

    // Reset all states
    int Reset(size_t inFreq, size_t outFreq, size_t num_channels);

    // Reset all states if any parameter has changed
    int ResetIfNeeded(size_t inFreq, size_t outFreq, size_t num_channels);

    // Resample samplesIn to samplesOut.
    int Push(const int16_t *samplesIn, size_t lengthIn, int16_t *samplesOut,
             size_t maxLen, size_t &outLen);  // NOLINT: to avoid changing APIs

private:
    enum ResamplerMode {
        kResamplerMode1To1,
        kResamplerMode1To2,
        kResamplerMode1To3,
        kResamplerMode1To4,
        kResamplerMode1To6,
        kResamplerMode1To12,
        kResamplerMode2To3,
        kResamplerMode2To11,
        kResamplerMode4To11,
        kResamplerMode8To11,
        kResamplerMode11To16,
        kResamplerMode11To32,
        kResamplerMode2To1,
        kResamplerMode3To1,
        kResamplerMode4To1,
        kResamplerMode6To1,
        kResamplerMode12To1,
        kResamplerMode3To2,
        kResamplerMode11To2,
        kResamplerMode11To4,
        kResamplerMode11To8
    };

    // Computes the resampler mode for a given sampling frequency pair.
    // Returns -1 for unsupported frequency pairs.
    static int ComputeResamplerMode(int in_freq_hz,
                                    int out_freq_hz,
                                    ResamplerMode *mode);

    // Generic pointers since we don't know what states we'll need
    void *state1_;
    void *state2_;
    void *state3_;

    // Storage if needed
    int16_t *in_buffer_;
    int16_t *out_buffer_;
    size_t in_buffer_size_;
    size_t out_buffer_size_;
    size_t in_buffer_size_max_;
    size_t out_buffer_size_max_;

    size_t my_in_frequency_khz_;
    size_t my_out_frequency_khz_;
    ResamplerMode my_mode_;
    size_t num_channels_;

    // Extra instance for stereo
    Resampler *slave_left_;
    Resampler *slave_right_;
};


#endif
