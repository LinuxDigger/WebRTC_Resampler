/*
 *  Copyright (c) 2011 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include <cstdlib>
#include <cstring>

#include "resampler.h"


// allpass filter coefficients.
static const int16_t kResampleAllpass[2][3] = {
        {821,  6110, 12382},
        {3050, 9368, 15063}
};

//
//   decimator
// input:  int32_t (shifted 15 positions to the left, + offset 16384) OVERWRITTEN!
// output: int16_t (saturated) (of length len/2)
// state:  filter state array; length = 8

void WebRtcSpl_DownBy2IntToShort(int32_t *in, int32_t len, int16_t *out,
                                 int32_t *state) {
    int32_t tmp0, tmp1, diff;
    int32_t i;

    len >>= 1;

    // lower allpass filter (operates on even input samples)
    for (i = 0; i < len; i++) {
        tmp0 = in[i << 1];
        diff = tmp0 - state[1];
        // scale down and round
        diff = (diff + (1 << 13)) >> 14;
        tmp1 = state[0] + diff * kResampleAllpass[1][0];
        state[0] = tmp0;
        diff = tmp1 - state[2];
        // scale down and truncate
        diff = diff >> 14;
        if (diff < 0)
            diff += 1;
        tmp0 = state[1] + diff * kResampleAllpass[1][1];
        state[1] = tmp1;
        diff = tmp0 - state[3];
        // scale down and truncate
        diff = diff >> 14;
        if (diff < 0)
            diff += 1;
        state[3] = state[2] + diff * kResampleAllpass[1][2];
        state[2] = tmp0;

        // divide by two and store temporarily
        in[i << 1] = (state[3] >> 1);
    }

    in++;

    // upper allpass filter (operates on odd input samples)
    for (i = 0; i < len; i++) {
        tmp0 = in[i << 1];
        diff = tmp0 - state[5];
        // scale down and round
        diff = (diff + (1 << 13)) >> 14;
        tmp1 = state[4] + diff * kResampleAllpass[0][0];
        state[4] = tmp0;
        diff = tmp1 - state[6];
        // scale down and round
        diff = diff >> 14;
        if (diff < 0)
            diff += 1;
        tmp0 = state[5] + diff * kResampleAllpass[0][1];
        state[5] = tmp1;
        diff = tmp0 - state[7];
        // scale down and truncate
        diff = diff >> 14;
        if (diff < 0)
            diff += 1;
        state[7] = state[6] + diff * kResampleAllpass[0][2];
        state[6] = tmp0;

        // divide by two and store temporarily
        in[i << 1] = (state[7] >> 1);
    }

    in--;

    // combine allpass outputs
    for (i = 0; i < len; i += 2) {
        // divide by two, add both allpass outputs and round
        tmp0 = (in[i << 1] + in[(i << 1) + 1]) >> 15;
        tmp1 = (in[(i << 1) + 2] + in[(i << 1) + 3]) >> 15;
        if (tmp0 > (int32_t) 0x00007FFF)
            tmp0 = 0x00007FFF;
        if (tmp0 < (int32_t) 0xFFFF8000)
            tmp0 = 0xFFFF8000;
        out[i] = (int16_t) tmp0;
        if (tmp1 > (int32_t) 0x00007FFF)
            tmp1 = 0x00007FFF;
        if (tmp1 < (int32_t) 0xFFFF8000)
            tmp1 = 0xFFFF8000;
        out[i + 1] = (int16_t) tmp1;
    }
}


//
//   interpolator
// input:  int16_t
// output: int32_t (normalized, not saturated) (of length len*2)
// state:  filter state array; length = 8
void WebRtcSpl_UpBy2ShortToInt(const int16_t *in, int32_t len, int32_t *out,
                               int32_t *state) {
    int32_t tmp0, tmp1, diff;
    int32_t i;

    // upper allpass filter (generates odd output samples)
    for (i = 0; i < len; i++) {
        tmp0 = ((int32_t) in[i] << 15) + (1 << 14);
        diff = tmp0 - state[5];
        // scale down and round
        diff = (diff + (1 << 13)) >> 14;
        tmp1 = state[4] + diff * kResampleAllpass[0][0];
        state[4] = tmp0;
        diff = tmp1 - state[6];
        // scale down and truncate
        diff = diff >> 14;
        if (diff < 0)
            diff += 1;
        tmp0 = state[5] + diff * kResampleAllpass[0][1];
        state[5] = tmp1;
        diff = tmp0 - state[7];
        // scale down and truncate
        diff = diff >> 14;
        if (diff < 0)
            diff += 1;
        state[7] = state[6] + diff * kResampleAllpass[0][2];
        state[6] = tmp0;

        // scale down, round and store
        out[i << 1] = state[7] >> 15;
    }

    out++;

    // lower allpass filter (generates even output samples)
    for (i = 0; i < len; i++) {
        tmp0 = ((int32_t) in[i] << 15) + (1 << 14);
        diff = tmp0 - state[1];
        // scale down and round
        diff = (diff + (1 << 13)) >> 14;
        tmp1 = state[0] + diff * kResampleAllpass[1][0];
        state[0] = tmp0;
        diff = tmp1 - state[2];
        // scale down and truncate
        diff = diff >> 14;
        if (diff < 0)
            diff += 1;
        tmp0 = state[1] + diff * kResampleAllpass[1][1];
        state[1] = tmp1;
        diff = tmp0 - state[3];
        // scale down and truncate
        diff = diff >> 14;
        if (diff < 0)
            diff += 1;
        state[3] = state[2] + diff * kResampleAllpass[1][2];
        state[2] = tmp0;

        // scale down, round and store
        out[i << 1] = state[3] >> 15;
    }
}


//
//   interpolator
// input:  int32_t (shifted 15 positions to the left, + offset 16384)
// output: int16_t (saturated) (of length len*2)
// state:  filter state array; length = 8
void WebRtcSpl_UpBy2IntToShort(const int32_t *in, int32_t len, int16_t *out,
                               int32_t *state) {
    int32_t tmp0, tmp1, diff;
    int32_t i;

    // upper allpass filter (generates odd output samples)
    for (i = 0; i < len; i++) {
        tmp0 = in[i];
        diff = tmp0 - state[5];
        // scale down and round
        diff = (diff + (1 << 13)) >> 14;
        tmp1 = state[4] + diff * kResampleAllpass[0][0];
        state[4] = tmp0;
        diff = tmp1 - state[6];
        // scale down and round
        diff = diff >> 14;
        if (diff < 0)
            diff += 1;
        tmp0 = state[5] + diff * kResampleAllpass[0][1];
        state[5] = tmp1;
        diff = tmp0 - state[7];
        // scale down and truncate
        diff = diff >> 14;
        if (diff < 0)
            diff += 1;
        state[7] = state[6] + diff * kResampleAllpass[0][2];
        state[6] = tmp0;

        // scale down, saturate and store
        tmp1 = state[7] >> 15;
        if (tmp1 > (int32_t) 0x00007FFF)
            tmp1 = 0x00007FFF;
        if (tmp1 < (int32_t) 0xFFFF8000)
            tmp1 = 0xFFFF8000;
        out[i << 1] = (int16_t) tmp1;
    }

    out++;

    // lower allpass filter (generates even output samples)
    for (i = 0; i < len; i++) {
        tmp0 = in[i];
        diff = tmp0 - state[1];
        // scale down and round
        diff = (diff + (1 << 13)) >> 14;
        tmp1 = state[0] + diff * kResampleAllpass[1][0];
        state[0] = tmp0;
        diff = tmp1 - state[2];
        // scale down and truncate
        diff = diff >> 14;
        if (diff < 0)
            diff += 1;
        tmp0 = state[1] + diff * kResampleAllpass[1][1];
        state[1] = tmp1;
        diff = tmp0 - state[3];
        // scale down and truncate
        diff = diff >> 14;
        if (diff < 0)
            diff += 1;
        state[3] = state[2] + diff * kResampleAllpass[1][2];
        state[2] = tmp0;

        // scale down, saturate and store
        tmp1 = state[3] >> 15;
        if (tmp1 > (int32_t) 0x00007FFF)
            tmp1 = 0x00007FFF;
        if (tmp1 < (int32_t) 0xFFFF8000)
            tmp1 = 0xFFFF8000;
        out[i << 1] = (int16_t) tmp1;
    }
}

//   lowpass filter
// input:  int16_t
// output: int32_t (normalized, not saturated)
// state:  filter state array; length = 8
void WebRtcSpl_LPBy2ShortToInt(const int16_t *in, int32_t len, int32_t *out,
                               int32_t *state) {
    int32_t tmp0, tmp1, diff;
    int32_t i;

    len >>= 1;

    // lower allpass filter: odd input -> even output samples
    in++;
    // initial state of polyphase delay element
    tmp0 = state[12];
    for (i = 0; i < len; i++) {
        diff = tmp0 - state[1];
        // scale down and round
        diff = (diff + (1 << 13)) >> 14;
        tmp1 = state[0] + diff * kResampleAllpass[1][0];
        state[0] = tmp0;
        diff = tmp1 - state[2];
        // scale down and truncate
        diff = diff >> 14;
        if (diff < 0)
            diff += 1;
        tmp0 = state[1] + diff * kResampleAllpass[1][1];
        state[1] = tmp1;
        diff = tmp0 - state[3];
        // scale down and truncate
        diff = diff >> 14;
        if (diff < 0)
            diff += 1;
        state[3] = state[2] + diff * kResampleAllpass[1][2];
        state[2] = tmp0;

        // scale down, round and store
        out[i << 1] = state[3] >> 1;
        tmp0 = ((int32_t) in[i << 1] << 15) + (1 << 14);
    }
    in--;

    // upper allpass filter: even input -> even output samples
    for (i = 0; i < len; i++) {
        tmp0 = ((int32_t) in[i << 1] << 15) + (1 << 14);
        diff = tmp0 - state[5];
        // scale down and round
        diff = (diff + (1 << 13)) >> 14;
        tmp1 = state[4] + diff * kResampleAllpass[0][0];
        state[4] = tmp0;
        diff = tmp1 - state[6];
        // scale down and round
        diff = diff >> 14;
        if (diff < 0)
            diff += 1;
        tmp0 = state[5] + diff * kResampleAllpass[0][1];
        state[5] = tmp1;
        diff = tmp0 - state[7];
        // scale down and truncate
        diff = diff >> 14;
        if (diff < 0)
            diff += 1;
        state[7] = state[6] + diff * kResampleAllpass[0][2];
        state[6] = tmp0;

        // average the two allpass outputs, scale down and store
        out[i << 1] = (out[i << 1] + (state[7] >> 1)) >> 15;
    }

    // switch to odd output samples
    out++;

    // lower allpass filter: even input -> odd output samples
    for (i = 0; i < len; i++) {
        tmp0 = ((int32_t) in[i << 1] << 15) + (1 << 14);
        diff = tmp0 - state[9];
        // scale down and round
        diff = (diff + (1 << 13)) >> 14;
        tmp1 = state[8] + diff * kResampleAllpass[1][0];
        state[8] = tmp0;
        diff = tmp1 - state[10];
        // scale down and truncate
        diff = diff >> 14;
        if (diff < 0)
            diff += 1;
        tmp0 = state[9] + diff * kResampleAllpass[1][1];
        state[9] = tmp1;
        diff = tmp0 - state[11];
        // scale down and truncate
        diff = diff >> 14;
        if (diff < 0)
            diff += 1;
        state[11] = state[10] + diff * kResampleAllpass[1][2];
        state[10] = tmp0;

        // scale down, round and store
        out[i << 1] = state[11] >> 1;
    }

    // upper allpass filter: odd input -> odd output samples
    in++;
    for (i = 0; i < len; i++) {
        tmp0 = ((int32_t) in[i << 1] << 15) + (1 << 14);
        diff = tmp0 - state[13];
        // scale down and round
        diff = (diff + (1 << 13)) >> 14;
        tmp1 = state[12] + diff * kResampleAllpass[0][0];
        state[12] = tmp0;
        diff = tmp1 - state[14];
        // scale down and round
        diff = diff >> 14;
        if (diff < 0)
            diff += 1;
        tmp0 = state[13] + diff * kResampleAllpass[0][1];
        state[13] = tmp1;
        diff = tmp0 - state[15];
        // scale down and truncate
        diff = diff >> 14;
        if (diff < 0)
            diff += 1;
        state[15] = state[14] + diff * kResampleAllpass[0][2];
        state[14] = tmp0;

        // average the two allpass outputs, scale down and store
        out[i << 1] = (out[i << 1] + (state[15] >> 1)) >> 15;
    }
}


static __inline int16_t WebRtcSpl_SatW32ToW16(int32_t value32) {
    int16_t out16 = (int16_t) value32;

    if (value32 > 32767)
        out16 = 32767;
    else if (value32 < -32768)
        out16 = -32768;

    return out16;
}

#ifdef WEBRTC_ARCH_ARM_V7

// allpass filter coefficients.
static const uint32_t kResampleAllpass1[3] = {3284, 24441, 49528 << 15};
static const uint32_t kResampleAllpass2[3] =
  {12199, 37471 << 15, 60255 << 15};

// Multiply two 32-bit values and accumulate to another input value.
// Return: state + ((diff * tbl_value) >> 16)

static __inline int32_t MUL_ACCUM_1(int32_t tbl_value,
                                    int32_t diff,
                                    int32_t state) {
  int32_t result;
  __asm __volatile ("smlawb %0, %1, %2, %3": "=r"(result): "r"(diff),
                                   "r"(tbl_value), "r"(state));
  return result;
}

// Multiply two 32-bit values and accumulate to another input value.
// Return: Return: state + (((diff << 1) * tbl_value) >> 32)
//
// The reason to introduce this function is that, in case we can't use smlawb
// instruction (in MUL_ACCUM_1) due to input value range, we can still use
// smmla to save some cycles.

static __inline int32_t MUL_ACCUM_2(int32_t tbl_value,
                                    int32_t diff,
                                    int32_t state) {
  int32_t result;
  __asm __volatile ("smmla %0, %1, %2, %3": "=r"(result): "r"(diff << 1),
                                  "r"(tbl_value), "r"(state));
  return result;
}

#else

// allpass filter coefficients.
static const uint16_t kResampleAllpass1[3] = {3284, 24441, 49528};
static const uint16_t kResampleAllpass2[3] = {12199, 37471, 60255};

// Multiply a 32-bit value with a 16-bit value and accumulate to another input:
#define MUL_ACCUM_1(a, b, c) WEBRTC_SPL_SCALEDIFF32(a, b, c)
#define MUL_ACCUM_2(a, b, c) WEBRTC_SPL_SCALEDIFF32(a, b, c)

#endif


void WebRtcSpl_DownsampleBy2(const int16_t *in, size_t len,
                             int16_t *out, int32_t *filtState) {
    int32_t tmp1, tmp2, diff, in32, out32;
    size_t i;

    register int32_t state0 = filtState[0];
    register int32_t state1 = filtState[1];
    register int32_t state2 = filtState[2];
    register int32_t state3 = filtState[3];
    register int32_t state4 = filtState[4];
    register int32_t state5 = filtState[5];
    register int32_t state6 = filtState[6];
    register int32_t state7 = filtState[7];

    for (i = (len >> 1); i > 0; i--) {
        // lower allpass filter
        in32 = (int32_t) (*in++) << 10;
        diff = in32 - state1;
        tmp1 = MUL_ACCUM_1(kResampleAllpass2[0], diff, state0);
        state0 = in32;
        diff = tmp1 - state2;
        tmp2 = MUL_ACCUM_2(kResampleAllpass2[1], diff, state1);
        state1 = tmp1;
        diff = tmp2 - state3;
        state3 = MUL_ACCUM_2(kResampleAllpass2[2], diff, state2);
        state2 = tmp2;

        // upper allpass filter
        in32 = (int32_t) (*in++) << 10;
        diff = in32 - state5;
        tmp1 = MUL_ACCUM_1(kResampleAllpass1[0], diff, state4);
        state4 = in32;
        diff = tmp1 - state6;
        tmp2 = MUL_ACCUM_1(kResampleAllpass1[1], diff, state5);
        state5 = tmp1;
        diff = tmp2 - state7;
        state7 = MUL_ACCUM_2(kResampleAllpass1[2], diff, state6);
        state6 = tmp2;

        // add two allpass outputs, divide by two and round
        out32 = (state3 + state7 + 1024) >> 11;

        // limit amplitude to prevent wrap-around, and write to output array
        *out++ = WebRtcSpl_SatW32ToW16(out32);
    }

    filtState[0] = state0;
    filtState[1] = state1;
    filtState[2] = state2;
    filtState[3] = state3;
    filtState[4] = state4;
    filtState[5] = state5;
    filtState[6] = state6;
    filtState[7] = state7;
}


void WebRtcSpl_UpsampleBy2(const int16_t *in, size_t len,
                           int16_t *out, int32_t *filtState) {
    int32_t tmp1, tmp2, diff, in32, out32;
    size_t i;

    register int32_t state0 = filtState[0];
    register int32_t state1 = filtState[1];
    register int32_t state2 = filtState[2];
    register int32_t state3 = filtState[3];
    register int32_t state4 = filtState[4];
    register int32_t state5 = filtState[5];
    register int32_t state6 = filtState[6];
    register int32_t state7 = filtState[7];

    for (i = len; i > 0; i--) {
        // lower allpass filter
        in32 = (int32_t) (*in++) << 10;
        diff = in32 - state1;
        tmp1 = MUL_ACCUM_1(kResampleAllpass1[0], diff, state0);
        state0 = in32;
        diff = tmp1 - state2;
        tmp2 = MUL_ACCUM_1(kResampleAllpass1[1], diff, state1);
        state1 = tmp1;
        diff = tmp2 - state3;
        state3 = MUL_ACCUM_2(kResampleAllpass1[2], diff, state2);
        state2 = tmp2;

        // round; limit amplitude to prevent wrap-around; write to output array
        out32 = (state3 + 512) >> 10;
        *out++ = WebRtcSpl_SatW32ToW16(out32);

        // upper allpass filter
        diff = in32 - state5;
        tmp1 = MUL_ACCUM_1(kResampleAllpass2[0], diff, state4);
        state4 = in32;
        diff = tmp1 - state6;
        tmp2 = MUL_ACCUM_2(kResampleAllpass2[1], diff, state5);
        state5 = tmp1;
        diff = tmp2 - state7;
        state7 = MUL_ACCUM_2(kResampleAllpass2[2], diff, state6);
        state6 = tmp2;

        // round; limit amplitude to prevent wrap-around; write to output array
        out32 = (state7 + 512) >> 10;
        *out++ = WebRtcSpl_SatW32ToW16(out32);
    }

    filtState[0] = state0;
    filtState[1] = state1;
    filtState[2] = state2;
    filtState[3] = state3;
    filtState[4] = state4;
    filtState[5] = state5;
    filtState[6] = state6;
    filtState[7] = state7;
}

/*File:resample_fractional.c
*  Copyright (c) 2011 The WebRTC project authors. All Rights Reserved.
*
*  Use of this source code is governed by a BSD-style license
*  that can be found in the LICENSE file in the root of the source
*  tree. An additional intellectual property rights grant can be found
*  in the file PATENTS.  All contributing project authors may
*  be found in the AUTHORS file in the root of the source tree.
*/


/*
* This file contains the resampling functions between 48, 44, 32 and 24 kHz.
* The description headers can be found in signal_processing_library.h
*
*/


// interpolation coefficients
static const int16_t kCoefficients48To32[2][8] = {
        {778, -2050, 1087,  23285, 12903, -3783, 441,   222},
        {222, 441,   -3783, 12903, 23285, 1087,  -2050, 778}
};

static const int16_t kCoefficients32To24[3][8] = {
        {767, -2362, 2434,  24406, 10620, -3838, 721,   90},
        {386, -381,  -2646, 19062, 19062, -2646, -381,  386},
        {90,  721,   -3838, 10620, 24406, 2434,  -2362, 767}
};

static const int16_t kCoefficients44To32[4][9] = {
        {117,  -669, 2245,  -6183, 26267, 13529, -3245, 845,  -138},
        {-101, 612,  -2283, 8532,  29790, -5138, 1789,  -524, 91},
        {50,   -292, 1016,  -3064, 32010, 3933,  -1147, 315,  -53},
        {-156, 974,  -3863, 18603, 21691, -6246, 2353,  -712, 126}
};

//   Resampling ratio: 2/3
// input:  int32_t (normalized, not saturated) :: size 3 * K
// output: int32_t (shifted 15 positions to the left, + offset 16384) :: size 2 * K
//      K: number of blocks

void WebRtcSpl_Resample48khzTo32khz(const int32_t *In, int32_t *Out, size_t K) {
    /////////////////////////////////////////////////////////////
    // Filter operation:
    //
    // Perform resampling (3 input samples -> 2 output samples);
    // process in sub blocks of size 3 samples.
    int32_t tmp;
    size_t m;

    for (m = 0; m < K; m++) {
        tmp = 1 << 14;
        tmp += kCoefficients48To32[0][0] * In[0];
        tmp += kCoefficients48To32[0][1] * In[1];
        tmp += kCoefficients48To32[0][2] * In[2];
        tmp += kCoefficients48To32[0][3] * In[3];
        tmp += kCoefficients48To32[0][4] * In[4];
        tmp += kCoefficients48To32[0][5] * In[5];
        tmp += kCoefficients48To32[0][6] * In[6];
        tmp += kCoefficients48To32[0][7] * In[7];
        Out[0] = tmp;

        tmp = 1 << 14;
        tmp += kCoefficients48To32[1][0] * In[1];
        tmp += kCoefficients48To32[1][1] * In[2];
        tmp += kCoefficients48To32[1][2] * In[3];
        tmp += kCoefficients48To32[1][3] * In[4];
        tmp += kCoefficients48To32[1][4] * In[5];
        tmp += kCoefficients48To32[1][5] * In[6];
        tmp += kCoefficients48To32[1][6] * In[7];
        tmp += kCoefficients48To32[1][7] * In[8];
        Out[1] = tmp;

        // update pointers
        In += 3;
        Out += 2;
    }
}

//   Resampling ratio: 3/4
// input:  int32_t (normalized, not saturated) :: size 4 * K
// output: int32_t (shifted 15 positions to the left, + offset 16384) :: size 3 * K
//      K: number of blocks

void WebRtcSpl_Resample32khzTo24khz(const int32_t *In, int32_t *Out, size_t K) {
    /////////////////////////////////////////////////////////////
    // Filter operation:
    //
    // Perform resampling (4 input samples -> 3 output samples);
    // process in sub blocks of size 4 samples.
    size_t m;
    int32_t tmp;

    for (m = 0; m < K; m++) {
        tmp = 1 << 14;
        tmp += kCoefficients32To24[0][0] * In[0];
        tmp += kCoefficients32To24[0][1] * In[1];
        tmp += kCoefficients32To24[0][2] * In[2];
        tmp += kCoefficients32To24[0][3] * In[3];
        tmp += kCoefficients32To24[0][4] * In[4];
        tmp += kCoefficients32To24[0][5] * In[5];
        tmp += kCoefficients32To24[0][6] * In[6];
        tmp += kCoefficients32To24[0][7] * In[7];
        Out[0] = tmp;

        tmp = 1 << 14;
        tmp += kCoefficients32To24[1][0] * In[1];
        tmp += kCoefficients32To24[1][1] * In[2];
        tmp += kCoefficients32To24[1][2] * In[3];
        tmp += kCoefficients32To24[1][3] * In[4];
        tmp += kCoefficients32To24[1][4] * In[5];
        tmp += kCoefficients32To24[1][5] * In[6];
        tmp += kCoefficients32To24[1][6] * In[7];
        tmp += kCoefficients32To24[1][7] * In[8];
        Out[1] = tmp;

        tmp = 1 << 14;
        tmp += kCoefficients32To24[2][0] * In[2];
        tmp += kCoefficients32To24[2][1] * In[3];
        tmp += kCoefficients32To24[2][2] * In[4];
        tmp += kCoefficients32To24[2][3] * In[5];
        tmp += kCoefficients32To24[2][4] * In[6];
        tmp += kCoefficients32To24[2][5] * In[7];
        tmp += kCoefficients32To24[2][6] * In[8];
        tmp += kCoefficients32To24[2][7] * In[9];
        Out[2] = tmp;

        // update pointers
        In += 4;
        Out += 3;
    }
}

//
// fractional resampling filters
//   Fout = 11/16 * Fin
//   Fout =  8/11 * Fin
//

// compute two inner-products and store them to output array
static void WebRtcSpl_ResampDotProduct(const int32_t *in1, const int32_t *in2,
                                       const int16_t *coef_ptr, int32_t *out1,
                                       int32_t *out2) {
    int32_t tmp1 = 16384;
    int32_t tmp2 = 16384;
    int16_t coef;

    coef = coef_ptr[0];
    tmp1 += coef * in1[0];
    tmp2 += coef * in2[-0];

    coef = coef_ptr[1];
    tmp1 += coef * in1[1];
    tmp2 += coef * in2[-1];

    coef = coef_ptr[2];
    tmp1 += coef * in1[2];
    tmp2 += coef * in2[-2];

    coef = coef_ptr[3];
    tmp1 += coef * in1[3];
    tmp2 += coef * in2[-3];

    coef = coef_ptr[4];
    tmp1 += coef * in1[4];
    tmp2 += coef * in2[-4];

    coef = coef_ptr[5];
    tmp1 += coef * in1[5];
    tmp2 += coef * in2[-5];

    coef = coef_ptr[6];
    tmp1 += coef * in1[6];
    tmp2 += coef * in2[-6];

    coef = coef_ptr[7];
    tmp1 += coef * in1[7];
    tmp2 += coef * in2[-7];

    coef = coef_ptr[8];
    *out1 = tmp1 + coef * in1[8];
    *out2 = tmp2 + coef * in2[-8];
}

//   Resampling ratio: 8/11
// input:  int32_t (normalized, not saturated) :: size 11 * K
// output: int32_t (shifted 15 positions to the left, + offset 16384) :: size  8 * K
//      K: number of blocks

void WebRtcSpl_Resample44khzTo32khz(const int32_t *In, int32_t *Out, size_t K) {
    /////////////////////////////////////////////////////////////
    // Filter operation:
    //
    // Perform resampling (11 input samples -> 8 output samples);
    // process in sub blocks of size 11 samples.
    int32_t tmp;
    size_t m;

    for (m = 0; m < K; m++) {
        tmp = 1 << 14;

        // first output sample
        Out[0] = ((int32_t) In[3] << 15) + tmp;

        // sum and accumulate filter coefficients and input samples
        tmp += kCoefficients44To32[3][0] * In[5];
        tmp += kCoefficients44To32[3][1] * In[6];
        tmp += kCoefficients44To32[3][2] * In[7];
        tmp += kCoefficients44To32[3][3] * In[8];
        tmp += kCoefficients44To32[3][4] * In[9];
        tmp += kCoefficients44To32[3][5] * In[10];
        tmp += kCoefficients44To32[3][6] * In[11];
        tmp += kCoefficients44To32[3][7] * In[12];
        tmp += kCoefficients44To32[3][8] * In[13];
        Out[4] = tmp;

        // sum and accumulate filter coefficients and input samples
        WebRtcSpl_ResampDotProduct(&In[0], &In[17], kCoefficients44To32[0], &Out[1], &Out[7]);

        // sum and accumulate filter coefficients and input samples
        WebRtcSpl_ResampDotProduct(&In[2], &In[15], kCoefficients44To32[1], &Out[2], &Out[6]);

        // sum and accumulate filter coefficients and input samples
        WebRtcSpl_ResampDotProduct(&In[3], &In[14], kCoefficients44To32[2], &Out[3], &Out[5]);

        // update pointers
        In += 11;
        Out += 8;
    }
}

/*File:resample_48khz
*  Copyright (c) 2011 The WebRTC project authors. All Rights Reserved.
*
*  Use of this source code is governed by a BSD-style license
*  that can be found in the LICENSE file in the root of the source
*  tree. An additional intellectual property rights grant can be found
*  in the file PATENTS.  All contributing project authors may
*  be found in the AUTHORS file in the root of the source tree.
*/


/*
* This file contains resampling functions between 48 kHz and nb/wb.
* The description header can be found in signal_processing_library.h
*
*/


////////////////////////////
///// 48 kHz -> 16 kHz /////
////////////////////////////

// 48 -> 16 resampler
void WebRtcSpl_Resample48khzTo16khz(const int16_t *in, int16_t *out,
                                    WebRtcSpl_State48khzTo16khz *state, int32_t *tmpmem) {
    ///// 48 --> 48(LP) /////
    // int16_t  in[480]
    // int32_t out[480]
    /////
    WebRtcSpl_LPBy2ShortToInt(in, 480, tmpmem + 16, state->S_48_48);

    ///// 48 --> 32 /////
    // int32_t  in[480]
    // int32_t out[320]
    /////
    // copy state to and from input array
    memcpy(tmpmem + 8, state->S_48_32, 8 * sizeof(int32_t));
    memcpy(state->S_48_32, tmpmem + 488, 8 * sizeof(int32_t));
    WebRtcSpl_Resample48khzTo32khz(tmpmem + 8, tmpmem, 160);

    ///// 32 --> 16 /////
    // int32_t  in[320]
    // int16_t out[160]
    /////
    WebRtcSpl_DownBy2IntToShort(tmpmem, 320, out, state->S_32_16);
}

// initialize state of 48 -> 16 resampler
void WebRtcSpl_ResetResample48khzTo16khz(WebRtcSpl_State48khzTo16khz *state) {
    memset(state->S_48_48, 0, 16 * sizeof(int32_t));
    memset(state->S_48_32, 0, 8 * sizeof(int32_t));
    memset(state->S_32_16, 0, 8 * sizeof(int32_t));
}

////////////////////////////
///// 16 kHz -> 48 kHz /////
////////////////////////////

// 16 -> 48 resampler
void WebRtcSpl_Resample16khzTo48khz(const int16_t *in, int16_t *out,
                                    WebRtcSpl_State16khzTo48khz *state, int32_t *tmpmem) {
    ///// 16 --> 32 /////
    // int16_t  in[160]
    // int32_t out[320]
    /////
    WebRtcSpl_UpBy2ShortToInt(in, 160, tmpmem + 16, state->S_16_32);

    ///// 32 --> 24 /////
    // int32_t  in[320]
    // int32_t out[240]
    // copy state to and from input array
    /////
    memcpy(tmpmem + 8, state->S_32_24, 8 * sizeof(int32_t));
    memcpy(state->S_32_24, tmpmem + 328, 8 * sizeof(int32_t));
    WebRtcSpl_Resample32khzTo24khz(tmpmem + 8, tmpmem, 80);

    ///// 24 --> 48 /////
    // int32_t  in[240]
    // int16_t out[480]
    /////
    WebRtcSpl_UpBy2IntToShort(tmpmem, 240, out, state->S_24_48);
}

// initialize state of 16 -> 48 resampler
void WebRtcSpl_ResetResample16khzTo48khz(WebRtcSpl_State16khzTo48khz *state) {
    memset(state->S_16_32, 0, 8 * sizeof(int32_t));
    memset(state->S_32_24, 0, 8 * sizeof(int32_t));
    memset(state->S_24_48, 0, 8 * sizeof(int32_t));
}

/* resample.c
*  Copyright (c) 2011 The WebRTC project authors. All Rights Reserved.
*
*  Use of this source code is governed by a BSD-style license
*  that can be found in the LICENSE file in the root of the source
*  tree. An additional intellectual property rights grant can be found
*  in the file PATENTS.  All contributing project authors may
*  be found in the AUTHORS file in the root of the source tree.
*/


/*
* This file contains the resampling functions for 22 kHz.
* The description header can be found in signal_processing_library.h
*
*/

// Declaration of internally used functions
static void WebRtcSpl_32khzTo22khzIntToShort(const int32_t *In, int16_t *Out,
                                             int32_t K);

void WebRtcSpl_32khzTo22khzIntToInt(const int32_t *In, int32_t *Out,
                                    int32_t K);

// interpolation coefficients
static const int16_t kCoefficients32To22[5][9] = {
        {127, -712, 2359,  -6333, 23456, 16775, -3695, 945,  -154},
        {-39, 230,  -830,  2785,  32366, -2324, 760,   -218, 38},
        {117, -663, 2222,  -6133, 26634, 13070, -3174, 831,  -137},
        {-77, 457,  -1677, 5958,  31175, -4136, 1405,  -408, 71},
        {98,  -560, 1900,  -5406, 29240, 9423,  -2480, 663,  -110}
};

//////////////////////
// 22 kHz -> 16 kHz //
//////////////////////

// number of subblocks; options: 1, 2, 4, 5, 10
#define SUB_BLOCKS_22_16    5

// 22 -> 16 resampler
void WebRtcSpl_Resample22khzTo16khz(const int16_t *in, int16_t *out,
                                    WebRtcSpl_State22khzTo16khz *state, int32_t *tmpmem) {
    int k;

    // process two blocks of 10/SUB_BLOCKS_22_16 ms (to reduce temp buffer size)
    for (k = 0; k < SUB_BLOCKS_22_16; k++) {
        ///// 22 --> 44 /////
        // int16_t  in[220/SUB_BLOCKS_22_16]
        // int32_t out[440/SUB_BLOCKS_22_16]
        /////
        WebRtcSpl_UpBy2ShortToInt(in, 220 / SUB_BLOCKS_22_16, tmpmem + 16, state->S_22_44);

        ///// 44 --> 32 /////
        // int32_t  in[440/SUB_BLOCKS_22_16]
        // int32_t out[320/SUB_BLOCKS_22_16]
        /////
        // copy state to and from input array
        tmpmem[8] = state->S_44_32[0];
        tmpmem[9] = state->S_44_32[1];
        tmpmem[10] = state->S_44_32[2];
        tmpmem[11] = state->S_44_32[3];
        tmpmem[12] = state->S_44_32[4];
        tmpmem[13] = state->S_44_32[5];
        tmpmem[14] = state->S_44_32[6];
        tmpmem[15] = state->S_44_32[7];
        state->S_44_32[0] = tmpmem[440 / SUB_BLOCKS_22_16 + 8];
        state->S_44_32[1] = tmpmem[440 / SUB_BLOCKS_22_16 + 9];
        state->S_44_32[2] = tmpmem[440 / SUB_BLOCKS_22_16 + 10];
        state->S_44_32[3] = tmpmem[440 / SUB_BLOCKS_22_16 + 11];
        state->S_44_32[4] = tmpmem[440 / SUB_BLOCKS_22_16 + 12];
        state->S_44_32[5] = tmpmem[440 / SUB_BLOCKS_22_16 + 13];
        state->S_44_32[6] = tmpmem[440 / SUB_BLOCKS_22_16 + 14];
        state->S_44_32[7] = tmpmem[440 / SUB_BLOCKS_22_16 + 15];

        WebRtcSpl_Resample44khzTo32khz(tmpmem + 8, tmpmem, 40 / SUB_BLOCKS_22_16);

        ///// 32 --> 16 /////
        // int32_t  in[320/SUB_BLOCKS_22_16]
        // int32_t out[160/SUB_BLOCKS_22_16]
        /////
        WebRtcSpl_DownBy2IntToShort(tmpmem, 320 / SUB_BLOCKS_22_16, out, state->S_32_16);

        // move input/output pointers 10/SUB_BLOCKS_22_16 ms seconds ahead
        in += 220 / SUB_BLOCKS_22_16;
        out += 160 / SUB_BLOCKS_22_16;
    }
}

// initialize state of 22 -> 16 resampler
void WebRtcSpl_ResetResample22khzTo16khz(WebRtcSpl_State22khzTo16khz *state) {
    int k;
    for (k = 0; k < 8; k++) {
        state->S_22_44[k] = 0;
        state->S_44_32[k] = 0;
        state->S_32_16[k] = 0;
    }
}

//////////////////////
// 16 kHz -> 22 kHz //
//////////////////////

// number of subblocks; options: 1, 2, 4, 5, 10
#define SUB_BLOCKS_16_22    4

// 16 -> 22 resampler
void WebRtcSpl_Resample16khzTo22khz(const int16_t *in, int16_t *out,
                                    WebRtcSpl_State16khzTo22khz *state, int32_t *tmpmem) {
    int k;

    // process two blocks of 10/SUB_BLOCKS_16_22 ms (to reduce temp buffer size)
    for (k = 0; k < SUB_BLOCKS_16_22; k++) {
        ///// 16 --> 32 /////
        // int16_t  in[160/SUB_BLOCKS_16_22]
        // int32_t out[320/SUB_BLOCKS_16_22]
        /////
        WebRtcSpl_UpBy2ShortToInt(in, 160 / SUB_BLOCKS_16_22, tmpmem + 8, state->S_16_32);

        ///// 32 --> 22 /////
        // int32_t  in[320/SUB_BLOCKS_16_22]
        // int32_t out[220/SUB_BLOCKS_16_22]
        /////
        // copy state to and from input array
        tmpmem[0] = state->S_32_22[0];
        tmpmem[1] = state->S_32_22[1];
        tmpmem[2] = state->S_32_22[2];
        tmpmem[3] = state->S_32_22[3];
        tmpmem[4] = state->S_32_22[4];
        tmpmem[5] = state->S_32_22[5];
        tmpmem[6] = state->S_32_22[6];
        tmpmem[7] = state->S_32_22[7];
        state->S_32_22[0] = tmpmem[320 / SUB_BLOCKS_16_22];
        state->S_32_22[1] = tmpmem[320 / SUB_BLOCKS_16_22 + 1];
        state->S_32_22[2] = tmpmem[320 / SUB_BLOCKS_16_22 + 2];
        state->S_32_22[3] = tmpmem[320 / SUB_BLOCKS_16_22 + 3];
        state->S_32_22[4] = tmpmem[320 / SUB_BLOCKS_16_22 + 4];
        state->S_32_22[5] = tmpmem[320 / SUB_BLOCKS_16_22 + 5];
        state->S_32_22[6] = tmpmem[320 / SUB_BLOCKS_16_22 + 6];
        state->S_32_22[7] = tmpmem[320 / SUB_BLOCKS_16_22 + 7];

        WebRtcSpl_32khzTo22khzIntToShort(tmpmem, out, 20 / SUB_BLOCKS_16_22);

        // move input/output pointers 10/SUB_BLOCKS_16_22 ms seconds ahead
        in += 160 / SUB_BLOCKS_16_22;
        out += 220 / SUB_BLOCKS_16_22;
    }
}

// initialize state of 16 -> 22 resampler
void WebRtcSpl_ResetResample16khzTo22khz(WebRtcSpl_State16khzTo22khz *state) {
    int k;
    for (k = 0; k < 8; k++) {
        state->S_16_32[k] = 0;
        state->S_32_22[k] = 0;
    }
}

//////////////////////
// 22 kHz ->  8 kHz //
//////////////////////

// number of subblocks; options: 1, 2, 5, 10
#define SUB_BLOCKS_22_8     2

// 22 -> 8 resampler
void WebRtcSpl_Resample22khzTo8khz(const int16_t *in, int16_t *out,
                                   WebRtcSpl_State22khzTo8khz *state, int32_t *tmpmem) {
    int k;

    // process two blocks of 10/SUB_BLOCKS_22_8 ms (to reduce temp buffer size)
    for (k = 0; k < SUB_BLOCKS_22_8; k++) {
        ///// 22 --> 22 lowpass /////
        // int16_t  in[220/SUB_BLOCKS_22_8]
        // int32_t out[220/SUB_BLOCKS_22_8]
        /////
        WebRtcSpl_LPBy2ShortToInt(in, 220 / SUB_BLOCKS_22_8, tmpmem + 16, state->S_22_22);

        ///// 22 --> 16 /////
        // int32_t  in[220/SUB_BLOCKS_22_8]
        // int32_t out[160/SUB_BLOCKS_22_8]
        /////
        // copy state to and from input array
        tmpmem[8] = state->S_22_16[0];
        tmpmem[9] = state->S_22_16[1];
        tmpmem[10] = state->S_22_16[2];
        tmpmem[11] = state->S_22_16[3];
        tmpmem[12] = state->S_22_16[4];
        tmpmem[13] = state->S_22_16[5];
        tmpmem[14] = state->S_22_16[6];
        tmpmem[15] = state->S_22_16[7];
        state->S_22_16[0] = tmpmem[220 / SUB_BLOCKS_22_8 + 8];
        state->S_22_16[1] = tmpmem[220 / SUB_BLOCKS_22_8 + 9];
        state->S_22_16[2] = tmpmem[220 / SUB_BLOCKS_22_8 + 10];
        state->S_22_16[3] = tmpmem[220 / SUB_BLOCKS_22_8 + 11];
        state->S_22_16[4] = tmpmem[220 / SUB_BLOCKS_22_8 + 12];
        state->S_22_16[5] = tmpmem[220 / SUB_BLOCKS_22_8 + 13];
        state->S_22_16[6] = tmpmem[220 / SUB_BLOCKS_22_8 + 14];
        state->S_22_16[7] = tmpmem[220 / SUB_BLOCKS_22_8 + 15];

        WebRtcSpl_Resample44khzTo32khz(tmpmem + 8, tmpmem, 20 / SUB_BLOCKS_22_8);

        ///// 16 --> 8 /////
        // int32_t in[160/SUB_BLOCKS_22_8]
        // int32_t out[80/SUB_BLOCKS_22_8]
        /////
        WebRtcSpl_DownBy2IntToShort(tmpmem, 160 / SUB_BLOCKS_22_8, out, state->S_16_8);

        // move input/output pointers 10/SUB_BLOCKS_22_8 ms seconds ahead
        in += 220 / SUB_BLOCKS_22_8;
        out += 80 / SUB_BLOCKS_22_8;
    }
}

// initialize state of 22 -> 8 resampler
void WebRtcSpl_ResetResample22khzTo8khz(WebRtcSpl_State22khzTo8khz *state) {
    int k;
    for (k = 0; k < 8; k++) {
        state->S_22_22[k] = 0;
        state->S_22_22[k + 8] = 0;
        state->S_22_16[k] = 0;
        state->S_16_8[k] = 0;
    }
}

//////////////////////
//  8 kHz -> 22 kHz //
//////////////////////

// number of subblocks; options: 1, 2, 5, 10
#define SUB_BLOCKS_8_22     2

// 8 -> 22 resampler
void WebRtcSpl_Resample8khzTo22khz(const int16_t *in, int16_t *out,
                                   WebRtcSpl_State8khzTo22khz *state, int32_t *tmpmem) {
    int k;

    // process two blocks of 10/SUB_BLOCKS_8_22 ms (to reduce temp buffer size)
    for (k = 0; k < SUB_BLOCKS_8_22; k++) {
        ///// 8 --> 16 /////
        // int16_t  in[80/SUB_BLOCKS_8_22]
        // int32_t out[160/SUB_BLOCKS_8_22]
        /////
        WebRtcSpl_UpBy2ShortToInt(in, 80 / SUB_BLOCKS_8_22, tmpmem + 18, state->S_8_16);

        ///// 16 --> 11 /////
        // int32_t  in[160/SUB_BLOCKS_8_22]
        // int32_t out[110/SUB_BLOCKS_8_22]
        /////
        // copy state to and from input array
        tmpmem[10] = state->S_16_11[0];
        tmpmem[11] = state->S_16_11[1];
        tmpmem[12] = state->S_16_11[2];
        tmpmem[13] = state->S_16_11[3];
        tmpmem[14] = state->S_16_11[4];
        tmpmem[15] = state->S_16_11[5];
        tmpmem[16] = state->S_16_11[6];
        tmpmem[17] = state->S_16_11[7];
        state->S_16_11[0] = tmpmem[160 / SUB_BLOCKS_8_22 + 10];
        state->S_16_11[1] = tmpmem[160 / SUB_BLOCKS_8_22 + 11];
        state->S_16_11[2] = tmpmem[160 / SUB_BLOCKS_8_22 + 12];
        state->S_16_11[3] = tmpmem[160 / SUB_BLOCKS_8_22 + 13];
        state->S_16_11[4] = tmpmem[160 / SUB_BLOCKS_8_22 + 14];
        state->S_16_11[5] = tmpmem[160 / SUB_BLOCKS_8_22 + 15];
        state->S_16_11[6] = tmpmem[160 / SUB_BLOCKS_8_22 + 16];
        state->S_16_11[7] = tmpmem[160 / SUB_BLOCKS_8_22 + 17];

        WebRtcSpl_32khzTo22khzIntToInt(tmpmem + 10, tmpmem, 10 / SUB_BLOCKS_8_22);

        ///// 11 --> 22 /////
        // int32_t  in[110/SUB_BLOCKS_8_22]
        // int16_t out[220/SUB_BLOCKS_8_22]
        /////
        WebRtcSpl_UpBy2IntToShort(tmpmem, 110 / SUB_BLOCKS_8_22, out, state->S_11_22);

        // move input/output pointers 10/SUB_BLOCKS_8_22 ms seconds ahead
        in += 80 / SUB_BLOCKS_8_22;
        out += 220 / SUB_BLOCKS_8_22;
    }
}

// initialize state of 8 -> 22 resampler
void WebRtcSpl_ResetResample8khzTo22khz(WebRtcSpl_State8khzTo22khz *state) {
    int k;
    for (k = 0; k < 8; k++) {
        state->S_8_16[k] = 0;
        state->S_16_11[k] = 0;
        state->S_11_22[k] = 0;
    }
}

// compute two inner-products and store them to output array
static void WebRtcSpl_DotProdIntToInt(const int32_t *in1, const int32_t *in2,
                                      const int16_t *coef_ptr, int32_t *out1,
                                      int32_t *out2) {
    int32_t tmp1 = 16384;
    int32_t tmp2 = 16384;
    int16_t coef;

    coef = coef_ptr[0];
    tmp1 += coef * in1[0];
    tmp2 += coef * in2[-0];

    coef = coef_ptr[1];
    tmp1 += coef * in1[1];
    tmp2 += coef * in2[-1];

    coef = coef_ptr[2];
    tmp1 += coef * in1[2];
    tmp2 += coef * in2[-2];

    coef = coef_ptr[3];
    tmp1 += coef * in1[3];
    tmp2 += coef * in2[-3];

    coef = coef_ptr[4];
    tmp1 += coef * in1[4];
    tmp2 += coef * in2[-4];

    coef = coef_ptr[5];
    tmp1 += coef * in1[5];
    tmp2 += coef * in2[-5];

    coef = coef_ptr[6];
    tmp1 += coef * in1[6];
    tmp2 += coef * in2[-6];

    coef = coef_ptr[7];
    tmp1 += coef * in1[7];
    tmp2 += coef * in2[-7];

    coef = coef_ptr[8];
    *out1 = tmp1 + coef * in1[8];
    *out2 = tmp2 + coef * in2[-8];
}

// compute two inner-products and store them to output array
static void WebRtcSpl_DotProdIntToShort(const int32_t *in1, const int32_t *in2,
                                        const int16_t *coef_ptr, int16_t *out1,
                                        int16_t *out2) {
    int32_t tmp1 = 16384;
    int32_t tmp2 = 16384;
    int16_t coef;

    coef = coef_ptr[0];
    tmp1 += coef * in1[0];
    tmp2 += coef * in2[-0];

    coef = coef_ptr[1];
    tmp1 += coef * in1[1];
    tmp2 += coef * in2[-1];

    coef = coef_ptr[2];
    tmp1 += coef * in1[2];
    tmp2 += coef * in2[-2];

    coef = coef_ptr[3];
    tmp1 += coef * in1[3];
    tmp2 += coef * in2[-3];

    coef = coef_ptr[4];
    tmp1 += coef * in1[4];
    tmp2 += coef * in2[-4];

    coef = coef_ptr[5];
    tmp1 += coef * in1[5];
    tmp2 += coef * in2[-5];

    coef = coef_ptr[6];
    tmp1 += coef * in1[6];
    tmp2 += coef * in2[-6];

    coef = coef_ptr[7];
    tmp1 += coef * in1[7];
    tmp2 += coef * in2[-7];

    coef = coef_ptr[8];
    tmp1 += coef * in1[8];
    tmp2 += coef * in2[-8];

    // scale down, round and saturate
    tmp1 >>= 15;
    if (tmp1 > (int32_t) 0x00007FFF)
        tmp1 = 0x00007FFF;
    if (tmp1 < (int32_t) 0xFFFF8000)
        tmp1 = 0xFFFF8000;
    tmp2 >>= 15;
    if (tmp2 > (int32_t) 0x00007FFF)
        tmp2 = 0x00007FFF;
    if (tmp2 < (int32_t) 0xFFFF8000)
        tmp2 = 0xFFFF8000;
    *out1 = (int16_t) tmp1;
    *out2 = (int16_t) tmp2;
}

//   Resampling ratio: 11/16
// input:  int32_t (normalized, not saturated) :: size 16 * K
// output: int32_t (shifted 15 positions to the left, + offset 16384) :: size 11 * K
//      K: Number of blocks

void WebRtcSpl_32khzTo22khzIntToInt(const int32_t *In,
                                    int32_t *Out,
                                    int32_t K) {
    /////////////////////////////////////////////////////////////
    // Filter operation:
    //
    // Perform resampling (16 input samples -> 11 output samples);
    // process in sub blocks of size 16 samples.
    int32_t m;

    for (m = 0; m < K; m++) {
        // first output sample
        Out[0] = ((int32_t) In[3] << 15) + (1 << 14);

        // sum and accumulate filter coefficients and input samples
        WebRtcSpl_DotProdIntToInt(&In[0], &In[22], kCoefficients32To22[0], &Out[1], &Out[10]);

        // sum and accumulate filter coefficients and input samples
        WebRtcSpl_DotProdIntToInt(&In[2], &In[20], kCoefficients32To22[1], &Out[2], &Out[9]);

        // sum and accumulate filter coefficients and input samples
        WebRtcSpl_DotProdIntToInt(&In[3], &In[19], kCoefficients32To22[2], &Out[3], &Out[8]);

        // sum and accumulate filter coefficients and input samples
        WebRtcSpl_DotProdIntToInt(&In[5], &In[17], kCoefficients32To22[3], &Out[4], &Out[7]);

        // sum and accumulate filter coefficients and input samples
        WebRtcSpl_DotProdIntToInt(&In[6], &In[16], kCoefficients32To22[4], &Out[5], &Out[6]);

        // update pointers
        In += 16;
        Out += 11;
    }
}

//   Resampling ratio: 11/16
// input:  int32_t (normalized, not saturated) :: size 16 * K
// output: int16_t (saturated) :: size 11 * K
//      K: Number of blocks

void WebRtcSpl_32khzTo22khzIntToShort(const int32_t *In,
                                      int16_t *Out,
                                      int32_t K) {
    /////////////////////////////////////////////////////////////
    // Filter operation:
    //
    // Perform resampling (16 input samples -> 11 output samples);
    // process in sub blocks of size 16 samples.
    int32_t tmp;
    int32_t m;

    for (m = 0; m < K; m++) {
        // first output sample
        tmp = In[3];
        if (tmp > (int32_t) 0x00007FFF)
            tmp = 0x00007FFF;
        if (tmp < (int32_t) 0xFFFF8000)
            tmp = 0xFFFF8000;
        Out[0] = (int16_t) tmp;

        // sum and accumulate filter coefficients and input samples
        WebRtcSpl_DotProdIntToShort(&In[0], &In[22], kCoefficients32To22[0], &Out[1], &Out[10]);

        // sum and accumulate filter coefficients and input samples
        WebRtcSpl_DotProdIntToShort(&In[2], &In[20], kCoefficients32To22[1], &Out[2], &Out[9]);

        // sum and accumulate filter coefficients and input samples
        WebRtcSpl_DotProdIntToShort(&In[3], &In[19], kCoefficients32To22[2], &Out[3], &Out[8]);

        // sum and accumulate filter coefficients and input samples
        WebRtcSpl_DotProdIntToShort(&In[5], &In[17], kCoefficients32To22[3], &Out[4], &Out[7]);

        // sum and accumulate filter coefficients and input samples
        WebRtcSpl_DotProdIntToShort(&In[6], &In[16], kCoefficients32To22[4], &Out[5], &Out[6]);

        // update pointers
        In += 16;
        Out += 11;
    }
}


Resampler::Resampler()
        : state1_(nullptr),
          state2_(nullptr),
          state3_(nullptr),
          in_buffer_(nullptr),
          out_buffer_(nullptr),
          in_buffer_size_(0),
          out_buffer_size_(0),
          in_buffer_size_max_(0),
          out_buffer_size_max_(0),
          my_in_frequency_khz_(0),
          my_out_frequency_khz_(0),
          my_mode_(kResamplerMode1To1),
          num_channels_(0),
          slave_left_(nullptr),
          slave_right_(nullptr) {
}

Resampler::Resampler(size_t inFreq, size_t outFreq, size_t num_channels)
        : Resampler() {
    Reset(inFreq, outFreq, num_channels);
}

Resampler::~Resampler() {
    if (state1_) {
        free(state1_);
    }
    if (state2_) {
        free(state2_);
    }
    if (state3_) {
        free(state3_);
    }
    if (in_buffer_) {
        free(in_buffer_);
    }
    if (out_buffer_) {
        free(out_buffer_);
    }

    delete slave_left_;
    delete slave_right_;

}

int Resampler::ResetIfNeeded(size_t inFreq, size_t outFreq, size_t num_channels) {
    size_t tmpInFreq_kHz = inFreq / 1000;
    size_t tmpOutFreq_kHz = outFreq / 1000;

    if ((tmpInFreq_kHz != my_in_frequency_khz_)
        || (tmpOutFreq_kHz != my_out_frequency_khz_)
        || (num_channels != num_channels_)) {
        return Reset(inFreq, outFreq, num_channels);
    } else {
        return 0;
    }
}

int Resampler::Reset(size_t inFreq, size_t outFreq, size_t num_channels) {
    if (num_channels != 1 && num_channels != 2) {
        printf("Reset() called with unsupported channel count, num_channels = %d .", num_channels);
        return -1;
    }
    ResamplerMode mode;
    if (ComputeResamplerMode(inFreq, outFreq, &mode) != 0) {
        printf("Reset() called with unsupported sample rates, inFreq = %d , outFreq = %d", inFreq, outFreq);
        return -1;
    }
    // Reinitialize internal state for the frequencies and sample rates.
    num_channels_ = num_channels;
    my_mode_ = mode;

    if (state1_) {
        free(state1_);
        state1_ = nullptr;
    }
    if (state2_) {
        free(state2_);
        state2_ = nullptr;
    }
    if (state3_) {
        free(state3_);
        state3_ = nullptr;
    }
    if (in_buffer_) {
        free(in_buffer_);
        in_buffer_ = nullptr;
    }
    if (out_buffer_) {
        free(out_buffer_);
        out_buffer_ = nullptr;
    }
    if (slave_left_) {
        delete slave_left_;
        slave_left_ = nullptr;
    }
    if (slave_right_) {
        delete slave_right_;
        slave_right_ = nullptr;
    }

    in_buffer_size_ = 0;
    out_buffer_size_ = 0;
    in_buffer_size_max_ = 0;
    out_buffer_size_max_ = 0;

    // We need to track what domain we're in.
    my_in_frequency_khz_ = inFreq / 1000;
    my_out_frequency_khz_ = outFreq / 1000;

    if (num_channels_ == 2) {
        // Create two mono resamplers.
        slave_left_ = new Resampler(inFreq, outFreq, 1);
        slave_right_ = new Resampler(inFreq, outFreq, 1);
    }

    // Now create the states we need.
    switch (my_mode_) {
        case kResamplerMode1To1:
            // No state needed;
            break;
        case kResamplerMode1To2:
            state1_ = malloc(8 * sizeof(int32_t));
            memset(state1_, 0, 8 * sizeof(int32_t));
            break;
        case kResamplerMode1To3:
            state1_ = malloc(sizeof(WebRtcSpl_State16khzTo48khz));
            WebRtcSpl_ResetResample16khzTo48khz(
                    static_cast<WebRtcSpl_State16khzTo48khz *>(state1_));
            break;
        case kResamplerMode1To4:
            // 1:2
            state1_ = malloc(8 * sizeof(int32_t));
            memset(state1_, 0, 8 * sizeof(int32_t));
            // 2:4
            state2_ = malloc(8 * sizeof(int32_t));
            memset(state2_, 0, 8 * sizeof(int32_t));
            break;
        case kResamplerMode1To6:
            // 1:2
            state1_ = malloc(8 * sizeof(int32_t));
            memset(state1_, 0, 8 * sizeof(int32_t));
            // 2:6
            state2_ = malloc(sizeof(WebRtcSpl_State16khzTo48khz));
            WebRtcSpl_ResetResample16khzTo48khz(
                    static_cast<WebRtcSpl_State16khzTo48khz *>(state2_));
            break;
        case kResamplerMode1To12:
            // 1:2
            state1_ = malloc(8 * sizeof(int32_t));
            memset(state1_, 0, 8 * sizeof(int32_t));
            // 2:4
            state2_ = malloc(8 * sizeof(int32_t));
            memset(state2_, 0, 8 * sizeof(int32_t));
            // 4:12
            state3_ = malloc(sizeof(WebRtcSpl_State16khzTo48khz));
            WebRtcSpl_ResetResample16khzTo48khz(
                    static_cast<WebRtcSpl_State16khzTo48khz *>(state3_));
            break;
        case kResamplerMode2To3:
            // 2:6
            state1_ = malloc(sizeof(WebRtcSpl_State16khzTo48khz));
            WebRtcSpl_ResetResample16khzTo48khz(
                    static_cast<WebRtcSpl_State16khzTo48khz *>(state1_));
            // 6:3
            state2_ = malloc(8 * sizeof(int32_t));
            memset(state2_, 0, 8 * sizeof(int32_t));
            break;
        case kResamplerMode2To11:
            state1_ = malloc(8 * sizeof(int32_t));
            memset(state1_, 0, 8 * sizeof(int32_t));

            state2_ = malloc(sizeof(WebRtcSpl_State8khzTo22khz));
            WebRtcSpl_ResetResample8khzTo22khz(
                    static_cast<WebRtcSpl_State8khzTo22khz *>(state2_));
            break;
        case kResamplerMode4To11:
            state1_ = malloc(sizeof(WebRtcSpl_State8khzTo22khz));
            WebRtcSpl_ResetResample8khzTo22khz(
                    static_cast<WebRtcSpl_State8khzTo22khz *>(state1_));
            break;
        case kResamplerMode8To11:
            state1_ = malloc(sizeof(WebRtcSpl_State16khzTo22khz));
            WebRtcSpl_ResetResample16khzTo22khz(
                    static_cast<WebRtcSpl_State16khzTo22khz *>(state1_));
            break;
        case kResamplerMode11To16:
            state1_ = malloc(8 * sizeof(int32_t));
            memset(state1_, 0, 8 * sizeof(int32_t));

            state2_ = malloc(sizeof(WebRtcSpl_State22khzTo16khz));
            WebRtcSpl_ResetResample22khzTo16khz(
                    static_cast<WebRtcSpl_State22khzTo16khz *>(state2_));
            break;
        case kResamplerMode11To32:
            // 11 -> 22
            state1_ = malloc(8 * sizeof(int32_t));
            memset(state1_, 0, 8 * sizeof(int32_t));

            // 22 -> 16
            state2_ = malloc(sizeof(WebRtcSpl_State22khzTo16khz));
            WebRtcSpl_ResetResample22khzTo16khz(
                    static_cast<WebRtcSpl_State22khzTo16khz *>(state2_));

            // 16 -> 32
            state3_ = malloc(8 * sizeof(int32_t));
            memset(state3_, 0, 8 * sizeof(int32_t));

            break;
        case kResamplerMode2To1:
            state1_ = malloc(8 * sizeof(int32_t));
            memset(state1_, 0, 8 * sizeof(int32_t));
            break;
        case kResamplerMode3To1:
            state1_ = malloc(sizeof(WebRtcSpl_State48khzTo16khz));
            WebRtcSpl_ResetResample48khzTo16khz(
                    static_cast<WebRtcSpl_State48khzTo16khz *>(state1_));
            break;
        case kResamplerMode4To1:
            // 4:2
            state1_ = malloc(8 * sizeof(int32_t));
            memset(state1_, 0, 8 * sizeof(int32_t));
            // 2:1
            state2_ = malloc(8 * sizeof(int32_t));
            memset(state2_, 0, 8 * sizeof(int32_t));
            break;
        case kResamplerMode6To1:
            // 6:2
            state1_ = malloc(sizeof(WebRtcSpl_State48khzTo16khz));
            WebRtcSpl_ResetResample48khzTo16khz(
                    static_cast<WebRtcSpl_State48khzTo16khz *>(state1_));
            // 2:1
            state2_ = malloc(8 * sizeof(int32_t));
            memset(state2_, 0, 8 * sizeof(int32_t));
            break;
        case kResamplerMode12To1:
            // 12:4
            state1_ = malloc(sizeof(WebRtcSpl_State48khzTo16khz));
            WebRtcSpl_ResetResample48khzTo16khz(
                    static_cast<WebRtcSpl_State48khzTo16khz *>(state1_));
            // 4:2
            state2_ = malloc(8 * sizeof(int32_t));
            memset(state2_, 0, 8 * sizeof(int32_t));
            // 2:1
            state3_ = malloc(8 * sizeof(int32_t));
            memset(state3_, 0, 8 * sizeof(int32_t));
            break;
        case kResamplerMode3To2:
            // 3:6
            state1_ = malloc(8 * sizeof(int32_t));
            memset(state1_, 0, 8 * sizeof(int32_t));
            // 6:2
            state2_ = malloc(sizeof(WebRtcSpl_State48khzTo16khz));
            WebRtcSpl_ResetResample48khzTo16khz(
                    static_cast<WebRtcSpl_State48khzTo16khz *>(state2_));
            break;
        case kResamplerMode11To2:
            state1_ = malloc(sizeof(WebRtcSpl_State22khzTo8khz));
            WebRtcSpl_ResetResample22khzTo8khz(
                    static_cast<WebRtcSpl_State22khzTo8khz *>(state1_));

            state2_ = malloc(8 * sizeof(int32_t));
            memset(state2_, 0, 8 * sizeof(int32_t));

            break;
        case kResamplerMode11To4:
            state1_ = malloc(sizeof(WebRtcSpl_State22khzTo8khz));
            WebRtcSpl_ResetResample22khzTo8khz(
                    static_cast<WebRtcSpl_State22khzTo8khz *>(state1_));
            break;
        case kResamplerMode11To8:
            state1_ = malloc(sizeof(WebRtcSpl_State22khzTo16khz));
            WebRtcSpl_ResetResample22khzTo16khz(
                    static_cast<WebRtcSpl_State22khzTo16khz *>(state1_));
            break;
    }

    return 0;
}

int Resampler::ComputeResamplerMode(int in_freq_hz,
                                    int out_freq_hz,
                                    ResamplerMode *mode) {
    // Start with a math exercise, Euclid's algorithm to find the gcd:
    int a = in_freq_hz;
    int b = out_freq_hz;
    int c = a % b;
    while (c != 0) {
        a = b;
        b = c;
        c = a % b;
    }
    // b is now the gcd;

    // Scale with GCD
    const int reduced_in_freq = in_freq_hz / b;
    const int reduced_out_freq = out_freq_hz / b;

    if (reduced_in_freq == reduced_out_freq) {
        *mode = kResamplerMode1To1;
    } else if (reduced_in_freq == 1) {
        switch (reduced_out_freq) {
            case 2:
                *mode = kResamplerMode1To2;
                break;
            case 3:
                *mode = kResamplerMode1To3;
                break;
            case 4:
                *mode = kResamplerMode1To4;
                break;
            case 6:
                *mode = kResamplerMode1To6;
                break;
            case 12:
                *mode = kResamplerMode1To12;
                break;
            default:
                return -1;
        }
    } else if (reduced_out_freq == 1) {
        switch (reduced_in_freq) {
            case 2:
                *mode = kResamplerMode2To1;
                break;
            case 3:
                *mode = kResamplerMode3To1;
                break;
            case 4:
                *mode = kResamplerMode4To1;
                break;
            case 6:
                *mode = kResamplerMode6To1;
                break;
            case 12:
                *mode = kResamplerMode12To1;
                break;
            default:
                return -1;
        }
    } else if ((reduced_in_freq == 2) && (reduced_out_freq == 3)) {
        *mode = kResamplerMode2To3;
    } else if ((reduced_in_freq == 2) && (reduced_out_freq == 11)) {
        *mode = kResamplerMode2To11;
    } else if ((reduced_in_freq == 4) && (reduced_out_freq == 11)) {
        *mode = kResamplerMode4To11;
    } else if ((reduced_in_freq == 8) && (reduced_out_freq == 11)) {
        *mode = kResamplerMode8To11;
    } else if ((reduced_in_freq == 3) && (reduced_out_freq == 2)) {
        *mode = kResamplerMode3To2;
    } else if ((reduced_in_freq == 11) && (reduced_out_freq == 2)) {
        *mode = kResamplerMode11To2;
    } else if ((reduced_in_freq == 11) && (reduced_out_freq == 4)) {
        *mode = kResamplerMode11To4;
    } else if ((reduced_in_freq == 11) && (reduced_out_freq == 16)) {
        *mode = kResamplerMode11To16;
    } else if ((reduced_in_freq == 11) && (reduced_out_freq == 32)) {
        *mode = kResamplerMode11To32;
    } else if ((reduced_in_freq == 11) && (reduced_out_freq == 8)) {
        *mode = kResamplerMode11To8;
    } else {
        return -1;
    }
    return 0;
}

// Synchronous resampling, all output samples are written to samplesOut
int Resampler::Push(const int16_t *samplesIn, size_t lengthIn,
                    int16_t *samplesOut, size_t maxLen, size_t &outLen) {
    if (num_channels_ == 2) {
        // Split up the signal and call the slave object for each channel
        int16_t *left =
                static_cast<int16_t *>(malloc(lengthIn * sizeof(int16_t) / 2));
        int16_t *right =
                static_cast<int16_t *>(malloc(lengthIn * sizeof(int16_t) / 2));
        int16_t *out_left =
                static_cast<int16_t *>(malloc(maxLen / 2 * sizeof(int16_t)));
        int16_t *out_right =
                static_cast<int16_t *>(malloc(maxLen / 2 * sizeof(int16_t)));
        if ((left == nullptr) || (right == nullptr) || (out_left == nullptr) || (out_right == nullptr)) {
            if (left) free(left);
            if (right) free(right);
            if (out_left) free(out_left);
            if (out_right) free(out_right);
            return -1;
        }
        int res = 0;
        for (size_t i = 0; i < lengthIn; i += 2) {
            left[i >> 1] = samplesIn[i];
            right[i >> 1] = samplesIn[i + 1];
        }

        // It's OK to overwrite the local parameter, since it's just a copy
        lengthIn = lengthIn / 2;

        size_t actualOutLen_left = 0;
        size_t actualOutLen_right = 0;
        // Do resampling for right channel
        res |= slave_left_->Push(left, lengthIn, out_left, maxLen / 2,
                                 actualOutLen_left);
        res |= slave_right_->Push(right, lengthIn, out_right, maxLen / 2,
                                  actualOutLen_right);
        if (res || (actualOutLen_left != actualOutLen_right)) {
            free(left);
            free(right);
            free(out_left);
            free(out_right);
            return -1;
        }

        // Reassemble the signal
        for (size_t i = 0; i < actualOutLen_left; i++) {
            samplesOut[i * 2] = out_left[i];
            samplesOut[i * 2 + 1] = out_right[i];
        }
        outLen = 2 * actualOutLen_left;

        free(left);
        free(right);
        free(out_left);
        free(out_right);

        return 0;
    }

    // Containers for temp samples
    int16_t *tmp;
    int16_t *tmp_2;
    // tmp data for resampling routines
    int32_t *tmp_mem;

    switch (my_mode_) {
        case kResamplerMode1To1:
            memcpy(samplesOut, samplesIn, lengthIn * sizeof(int16_t));
            outLen = lengthIn;
            break;
        case kResamplerMode1To2:
            if (maxLen < (lengthIn * 2)) {
                return -1;
            }
            WebRtcSpl_UpsampleBy2(samplesIn, lengthIn, samplesOut,
                                  static_cast<int32_t *>(state1_));
            outLen = lengthIn * 2;
            return 0;
        case kResamplerMode1To3:

            // We can only handle blocks of 160 samples
            // Can be fixed, but I don't think it's needed
            if ((lengthIn % 160) != 0) {
                return -1;
            }
            if (maxLen < (lengthIn * 3)) {
                return -1;
            }
            tmp_mem = static_cast<int32_t *>(malloc(336 * sizeof(int32_t)));
            if (tmp_mem == nullptr) {
                return -1;
            }
            for (size_t i = 0; i < lengthIn; i += 160) {
                WebRtcSpl_Resample16khzTo48khz(
                        samplesIn + i, samplesOut + i * 3,
                        static_cast<WebRtcSpl_State16khzTo48khz *>(state1_), tmp_mem);
            }
            outLen = lengthIn * 3;
            free(tmp_mem);
            return 0;
        case kResamplerMode1To4:
            if (maxLen < (lengthIn * 4)) {
                return -1;
            }

            tmp = static_cast<int16_t *>(malloc(sizeof(int16_t) * 2 * lengthIn));
            // 1:2
            WebRtcSpl_UpsampleBy2(samplesIn, lengthIn, tmp,
                                  static_cast<int32_t *>(state1_));
            // 2:4
            WebRtcSpl_UpsampleBy2(tmp, lengthIn * 2, samplesOut,
                                  static_cast<int32_t *>(state2_));
            outLen = lengthIn * 4;
            free(tmp);
            return 0;
        case kResamplerMode1To6:
            // We can only handle blocks of 80 samples
            // Can be fixed, but I don't think it's needed
            if ((lengthIn % 80) != 0) {
                return -1;
            }
            if (maxLen < (lengthIn * 6)) {
                return -1;
            }

            // 1:2

            tmp_mem = static_cast<int32_t *>(malloc(336 * sizeof(int32_t)));
            tmp = static_cast<int16_t *>(malloc(sizeof(int16_t) * 2 * lengthIn));
            if (tmp_mem == nullptr || tmp == nullptr) {
                if (tmp_mem)
                    free(tmp_mem);
                if (tmp)
                    free(tmp);
                return -1;
            }
            WebRtcSpl_UpsampleBy2(samplesIn, lengthIn, tmp,
                                  static_cast<int32_t *>(state1_));
            outLen = lengthIn * 2;

            for (size_t i = 0; i < outLen; i += 160) {
                WebRtcSpl_Resample16khzTo48khz(
                        tmp + i, samplesOut + i * 3,
                        static_cast<WebRtcSpl_State16khzTo48khz *>(state2_), tmp_mem);
            }
            outLen = outLen * 3;
            free(tmp_mem);
            free(tmp);

            return 0;
        case kResamplerMode1To12:
            // We can only handle blocks of 40 samples
            // Can be fixed, but I don't think it's needed
            if ((lengthIn % 40) != 0) {
                return -1;
            }
            if (maxLen < (lengthIn * 12)) {
                return -1;
            }

            tmp_mem = static_cast<int32_t *>(malloc(336 * sizeof(int32_t)));
            tmp = static_cast<int16_t *>(malloc(sizeof(int16_t) * 4 * lengthIn));
            if (tmp_mem == nullptr || tmp == nullptr) {
                if (tmp_mem)
                    free(tmp_mem);
                if (tmp)
                    free(tmp);
                return -1;
            }  // 1:2
            WebRtcSpl_UpsampleBy2(samplesIn, lengthIn, samplesOut,
                                  static_cast<int32_t *>(state1_));
            outLen = lengthIn * 2;
            // 2:4
            WebRtcSpl_UpsampleBy2(samplesOut, outLen, tmp,
                                  static_cast<int32_t *>(state2_));
            outLen = outLen * 2;
            // 4:12
            for (size_t i = 0; i < outLen; i += 160) {
                // WebRtcSpl_Resample16khzTo48khz() takes a block of 160 samples
                // as input and outputs a resampled block of 480 samples. The
                // data is now actually in 32 kHz sampling rate, despite the
                // function name, and with a resampling factor of three becomes
                // 96 kHz.
                WebRtcSpl_Resample16khzTo48khz(
                        tmp + i, samplesOut + i * 3,
                        static_cast<WebRtcSpl_State16khzTo48khz *>(state3_), tmp_mem);
            }
            outLen = outLen * 3;
            free(tmp_mem);
            free(tmp);

            return 0;
        case kResamplerMode2To3:
            if (maxLen < (lengthIn * 3 / 2)) {
                return -1;
            }
            // 2:6
            // We can only handle blocks of 160 samples
            // Can be fixed, but I don't think it's needed
            if ((lengthIn % 160) != 0) {
                return -1;
            }
            tmp = static_cast<int16_t *> (malloc(sizeof(int16_t) * lengthIn * 3));
            tmp_mem = static_cast<int32_t *>(malloc(336 * sizeof(int32_t)));
            if (tmp_mem == nullptr || tmp == nullptr) {
                if (tmp_mem)
                    free(tmp_mem);
                if (tmp)
                    free(tmp);
                return -1;
            }
            for (size_t i = 0; i < lengthIn; i += 160) {
                WebRtcSpl_Resample16khzTo48khz(
                        samplesIn + i, tmp + i * 3,
                        static_cast<WebRtcSpl_State16khzTo48khz *>(state1_), tmp_mem);
            }
            lengthIn = lengthIn * 3;
            // 6:3
            WebRtcSpl_DownsampleBy2(tmp, lengthIn, samplesOut,
                                    static_cast<int32_t *>(state2_));
            outLen = lengthIn / 2;
            free(tmp);
            free(tmp_mem);
            return 0;
        case kResamplerMode2To11:

            // We can only handle blocks of 80 samples
            // Can be fixed, but I don't think it's needed
            if ((lengthIn % 80) != 0) {
                return -1;
            }
            if (maxLen < ((lengthIn * 11) / 2)) {
                return -1;
            }
            tmp = static_cast<int16_t *>(malloc(sizeof(int16_t) * 2 * lengthIn));
            tmp_mem = static_cast<int32_t *>(malloc(98 * sizeof(int32_t)));
            if (tmp_mem == nullptr || tmp == nullptr) {
                if (tmp_mem)
                    free(tmp_mem);
                if (tmp)
                    free(tmp);
                return -1;
            }
            // 1:2
            WebRtcSpl_UpsampleBy2(samplesIn, lengthIn, tmp,
                                  static_cast<int32_t *>(state1_));
            lengthIn *= 2;
            for (size_t i = 0; i < lengthIn; i += 80) {
                WebRtcSpl_Resample8khzTo22khz(
                        tmp + i, samplesOut + (i * 11) / 4,
                        static_cast<WebRtcSpl_State8khzTo22khz *>(state2_), tmp_mem);
            }
            outLen = (lengthIn * 11) / 4;
            free(tmp_mem);
            free(tmp);
            return 0;
        case kResamplerMode4To11:

            // We can only handle blocks of 80 samples
            // Can be fixed, but I don't think it's needed
            if ((lengthIn % 80) != 0) {
                return -1;
            }
            if (maxLen < ((lengthIn * 11) / 4)) {
                return -1;
            }
            tmp_mem = static_cast<int32_t *>(malloc(98 * sizeof(int32_t)));
            if (tmp_mem == nullptr) {
                return -1;
            }
            for (size_t i = 0; i < lengthIn; i += 80) {
                WebRtcSpl_Resample8khzTo22khz(
                        samplesIn + i, samplesOut + (i * 11) / 4,
                        static_cast<WebRtcSpl_State8khzTo22khz *>(state1_), tmp_mem);
            }
            outLen = (lengthIn * 11) / 4;
            free(tmp_mem);
            return 0;
        case kResamplerMode8To11:
            // We can only handle blocks of 160 samples
            // Can be fixed, but I don't think it's needed
            if ((lengthIn % 160) != 0) {
                return -1;
            }
            if (maxLen < ((lengthIn * 11) / 8)) {
                return -1;
            }
            tmp_mem = static_cast<int32_t *>(malloc(88 * sizeof(int32_t)));
            if (tmp_mem == nullptr) {
                return -1;
            }
            for (size_t i = 0; i < lengthIn; i += 160) {
                WebRtcSpl_Resample16khzTo22khz(
                        samplesIn + i, samplesOut + (i * 11) / 8,
                        static_cast<WebRtcSpl_State16khzTo22khz *>(state1_), tmp_mem);
            }
            outLen = (lengthIn * 11) / 8;
            free(tmp_mem);
            return 0;

        case kResamplerMode11To16:
            // We can only handle blocks of 110 samples
            if ((lengthIn % 110) != 0) {
                return -1;
            }
            if (maxLen < ((lengthIn * 16) / 11)) {
                return -1;
            }

            tmp_mem = static_cast<int32_t *>(malloc(104 * sizeof(int32_t)));
            tmp = static_cast<int16_t *>(malloc((sizeof(int16_t) * lengthIn * 2)));
            if (tmp_mem == nullptr || tmp == nullptr) {
                if (tmp_mem)
                    free(tmp_mem);
                if (tmp)
                    free(tmp);
                return -1;
            }
            WebRtcSpl_UpsampleBy2(samplesIn, lengthIn, tmp,
                                  static_cast<int32_t *>(state1_));

            for (size_t i = 0; i < (lengthIn * 2); i += 220) {
                WebRtcSpl_Resample22khzTo16khz(
                        tmp + i, samplesOut + (i / 220) * 160,
                        static_cast<WebRtcSpl_State22khzTo16khz *>(state2_), tmp_mem);
            }

            outLen = (lengthIn * 16) / 11;

            free(tmp_mem);
            free(tmp);
            return 0;

        case kResamplerMode11To32:

            // We can only handle blocks of 110 samples
            if ((lengthIn % 110) != 0) {
                return -1;
            }
            if (maxLen < ((lengthIn * 32) / 11)) {
                return -1;
            }

            tmp_mem = static_cast<int32_t *>(malloc(104 * sizeof(int32_t)));
            tmp = static_cast<int16_t *>(malloc((sizeof(int16_t) * lengthIn * 2)));
            if (tmp_mem == nullptr || tmp == nullptr) {
                if (tmp_mem)
                    free(tmp_mem);
                if (tmp)
                    free(tmp);
                return -1;
            }
            // 11 -> 22 kHz in samplesOut
            WebRtcSpl_UpsampleBy2(samplesIn, lengthIn, samplesOut,
                                  static_cast<int32_t *>(state1_));

            // 22 -> 16 in tmp
            for (size_t i = 0; i < (lengthIn * 2); i += 220) {
                WebRtcSpl_Resample22khzTo16khz(
                        samplesOut + i, tmp + (i / 220) * 160,
                        static_cast<WebRtcSpl_State22khzTo16khz *>(state2_), tmp_mem);
            }

            // 16 -> 32 in samplesOut
            WebRtcSpl_UpsampleBy2(tmp, (lengthIn * 16) / 11, samplesOut,
                                  static_cast<int32_t *>(state3_));

            outLen = (lengthIn * 32) / 11;

            free(tmp_mem);
            free(tmp);
            return 0;

        case kResamplerMode2To1:
            if (maxLen < (lengthIn / 2)) {
                return -1;
            }
            WebRtcSpl_DownsampleBy2(samplesIn, lengthIn, samplesOut,
                                    static_cast<int32_t *>(state1_));
            outLen = lengthIn / 2;
            return 0;
        case kResamplerMode3To1:
            // We can only handle blocks of 480 samples
            // Can be fixed, but I don't think it's needed
            if ((lengthIn % 480) != 0) {
                return -1;
            }
            if (maxLen < (lengthIn / 3)) {
                return -1;
            }
            tmp_mem = static_cast<int32_t *>(malloc(496 * sizeof(int32_t)));
            if (tmp_mem == nullptr) {
                return -1;
            }
            for (size_t i = 0; i < lengthIn; i += 480) {
                WebRtcSpl_Resample48khzTo16khz(
                        samplesIn + i, samplesOut + i / 3,
                        static_cast<WebRtcSpl_State48khzTo16khz *>(state1_), tmp_mem);
            }
            outLen = lengthIn / 3;
            free(tmp_mem);
            return 0;
        case kResamplerMode4To1:
            if (maxLen < (lengthIn / 4)) {
                return -1;
            }
            tmp = static_cast<int16_t *>(malloc(sizeof(int16_t) * lengthIn / 2));
            if (tmp == nullptr) {
                return -1;
            }    // 4:2
            WebRtcSpl_DownsampleBy2(samplesIn, lengthIn, tmp,
                                    static_cast<int32_t *>(state1_));
            // 2:1
            WebRtcSpl_DownsampleBy2(tmp, lengthIn / 2, samplesOut,
                                    static_cast<int32_t *>(state2_));
            outLen = lengthIn / 4;
            free(tmp);
            return 0;

        case kResamplerMode6To1:
            // We can only handle blocks of 480 samples
            // Can be fixed, but I don't think it's needed
            if ((lengthIn % 480) != 0) {
                return -1;
            }
            if (maxLen < (lengthIn / 6)) {
                return -1;
            }

            tmp_mem = static_cast<int32_t *>(malloc(496 * sizeof(int32_t)));
            tmp = static_cast<int16_t *>(malloc((sizeof(int16_t) * lengthIn) / 3));
            if (tmp_mem == nullptr || tmp == nullptr) {
                if (tmp_mem)
                    free(tmp_mem);
                if (tmp)
                    free(tmp);
                return -1;
            }
            for (size_t i = 0; i < lengthIn; i += 480) {
                WebRtcSpl_Resample48khzTo16khz(
                        samplesIn + i, tmp + i / 3,
                        static_cast<WebRtcSpl_State48khzTo16khz *>(state1_), tmp_mem);
            }
            outLen = lengthIn / 3;
            free(tmp_mem);
            WebRtcSpl_DownsampleBy2(tmp, outLen, samplesOut,
                                    static_cast<int32_t *>(state2_));
            free(tmp);
            outLen = outLen / 2;
            return 0;
        case kResamplerMode12To1:
            // We can only handle blocks of 480 samples
            // Can be fixed, but I don't think it's needed
            if ((lengthIn % 480) != 0) {
                return -1;
            }
            if (maxLen < (lengthIn / 12)) {
                return -1;
            }

            tmp_mem = static_cast<int32_t *>(malloc(496 * sizeof(int32_t)));
            tmp = static_cast<int16_t *>(malloc((sizeof(int16_t) * lengthIn) / 3));
            tmp_2 = static_cast<int16_t *>(malloc((sizeof(int16_t) * lengthIn) / 6));
            if (tmp_mem == nullptr || tmp_2 == nullptr || tmp == nullptr) {
                if (tmp_mem)
                    free(tmp_mem);
                if (tmp)
                    free(tmp);
                if (tmp_2)
                    free(tmp_2);
                return -1;
            }    // 12:4
            for (size_t i = 0; i < lengthIn; i += 480) {
                // WebRtcSpl_Resample48khzTo16khz() takes a block of 480 samples
                // as input and outputs a resampled block of 160 samples. The
                // data is now actually in 96 kHz sampling rate, despite the
                // function name, and with a resampling factor of 1/3 becomes
                // 32 kHz.
                WebRtcSpl_Resample48khzTo16khz(
                        samplesIn + i, tmp + i / 3,
                        static_cast<WebRtcSpl_State48khzTo16khz *>(state1_), tmp_mem);
            }
            outLen = lengthIn / 3;
            free(tmp_mem);
            // 4:2
            WebRtcSpl_DownsampleBy2(tmp, outLen, tmp_2,
                                    static_cast<int32_t *>(state2_));
            outLen = outLen / 2;
            free(tmp);
            // 2:1
            WebRtcSpl_DownsampleBy2(tmp_2, outLen, samplesOut,
                                    static_cast<int32_t *>(state3_));
            free(tmp_2);
            outLen = outLen / 2;
            return 0;
        case kResamplerMode3To2:
            if (maxLen < (lengthIn * 2 / 3)) {
                return -1;
            }
            // 3:6
            tmp = static_cast<int16_t *> (malloc(sizeof(int16_t) * lengthIn * 2));
            if (tmp == nullptr) {
                return -1;
            }
            WebRtcSpl_UpsampleBy2(samplesIn, lengthIn, tmp,
                                  static_cast<int32_t *>(state1_));
            lengthIn *= 2;
            // 6:2
            // We can only handle blocks of 480 samples
            // Can be fixed, but I don't think it's needed
            if ((lengthIn % 480) != 0) {
                free(tmp);
                return -1;
            }
            tmp_mem = static_cast<int32_t *>(malloc(496 * sizeof(int32_t)));
            if (tmp_mem == nullptr) {
                return -1;
            }
            for (size_t i = 0; i < lengthIn; i += 480) {
                WebRtcSpl_Resample48khzTo16khz(
                        tmp + i, samplesOut + i / 3,
                        static_cast<WebRtcSpl_State48khzTo16khz *>(state2_), tmp_mem);
            }
            outLen = lengthIn / 3;
            free(tmp);
            free(tmp_mem);
            return 0;
        case kResamplerMode11To2:
            // We can only handle blocks of 220 samples
            // Can be fixed, but I don't think it's needed
            if ((lengthIn % 220) != 0) {
                return -1;
            }
            if (maxLen < ((lengthIn * 2) / 11)) {
                return -1;
            }
            tmp_mem = static_cast<int32_t *>(malloc(126 * sizeof(int32_t)));
            tmp = static_cast<int16_t *>(
                    malloc((lengthIn * 4) / 11 * sizeof(int16_t)));
            if (tmp_mem == nullptr || tmp == nullptr) {
                if (tmp)
                    free(tmp);
                if (tmp_mem)
                    free(tmp_mem);
                return -1;
            }
            for (size_t i = 0; i < lengthIn; i += 220) {
                WebRtcSpl_Resample22khzTo8khz(
                        samplesIn + i, tmp + (i * 4) / 11,
                        static_cast<WebRtcSpl_State22khzTo8khz *>(state1_), tmp_mem);
            }
            lengthIn = (lengthIn * 4) / 11;

            WebRtcSpl_DownsampleBy2(tmp, lengthIn, samplesOut,
                                    static_cast<int32_t *>(state2_));
            outLen = lengthIn / 2;

            free(tmp_mem);
            free(tmp);
            return 0;
        case kResamplerMode11To4:
            // We can only handle blocks of 220 samples
            // Can be fixed, but I don't think it's needed
            if ((lengthIn % 220) != 0) {
                return -1;
            }
            if (maxLen < ((lengthIn * 4) / 11)) {
                return -1;
            }
            tmp_mem = static_cast<int32_t *>(malloc(126 * sizeof(int32_t)));
            if (tmp_mem == nullptr) {
                return -1;
            }
            for (size_t i = 0; i < lengthIn; i += 220) {
                WebRtcSpl_Resample22khzTo8khz(
                        samplesIn + i, samplesOut + (i * 4) / 11,
                        static_cast<WebRtcSpl_State22khzTo8khz *>(state1_), tmp_mem);
            }
            outLen = (lengthIn * 4) / 11;
            free(tmp_mem);
            return 0;
        case kResamplerMode11To8:
            // We can only handle blocks of 160 samples
            // Can be fixed, but I don't think it's needed
            if ((lengthIn % 220) != 0) {
                return -1;
            }
            if (maxLen < ((lengthIn * 8) / 11)) {
                return -1;
            }
            tmp_mem = static_cast<int32_t *>(malloc(104 * sizeof(int32_t)));
            if (tmp_mem == nullptr) {
                return -1;
            }
            for (size_t i = 0; i < lengthIn; i += 220) {
                WebRtcSpl_Resample22khzTo16khz(
                        samplesIn + i, samplesOut + (i * 8) / 11,
                        static_cast<WebRtcSpl_State22khzTo16khz *>(state1_), tmp_mem);
            }
            outLen = (lengthIn * 8) / 11;
            free(tmp_mem);
            return 0;
            break;
    }
    return 0;
}
