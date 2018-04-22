#include <cstdio>
#include <cstdlib>
#include <cstdint>
//采用https://github.com/mackron/dr_libs/blob/master/dr_wav.h 解码
#define DR_WAV_IMPLEMENTATION

#include "dr_wav.h"
#include "resampler.h"

//写wav文件
void wavWrite_int16(char *filename, int16_t *buffer, size_t sampleRate, size_t totalSampleCount) {
    drwav_data_format format = {};
    format.container = drwav_container_riff;     // <-- drwav_container_riff = normal WAV files, drwav_container_w64 = Sony Wave64.
    format.format = DR_WAVE_FORMAT_PCM;          // <-- Any of the DR_WAVE_FORMAT_* codes.
    format.channels = 1;
    format.sampleRate = (drwav_uint32) sampleRate;
    format.bitsPerSample = 16;
    drwav *pWav = drwav_open_file_write(filename, &format);
    if (pWav) {
        drwav_uint64 samplesWritten = drwav_write(pWav, totalSampleCount, buffer);
        drwav_uninit(pWav);
        if (samplesWritten != totalSampleCount) {
            fprintf(stderr, "ERROR\n");
            exit(1);
        }
    }
}

//读取wav文件
int16_t *wavRead_int16(char *filename, uint32_t *sampleRate, uint64_t *totalSampleCount) {
    unsigned int channels;
    int16_t *buffer = drwav_open_and_read_file_s16(filename, &channels, sampleRate, totalSampleCount);
    if (buffer == nullptr) {
        printf("读取wav文件失败.");
    }
    //仅仅处理单通道音频
    if (channels != 1) {
        drwav_free(buffer);
        buffer = nullptr;
        *sampleRate = 0;
        *totalSampleCount = 0;
    }
    return buffer;
}

//分割路径函数
void splitpath(const char *path, char *drv, char *dir, char *name, char *ext) {
    const char *end;
    const char *p;
    const char *s;
    if (path[0] && path[1] == ':') {
        if (drv) {
            *drv++ = *path++;
            *drv++ = *path++;
            *drv = '\0';
        }
    } else if (drv)
        *drv = '\0';
    for (end = path; *end && *end != ':';)
        end++;
    for (p = end; p > path && *--p != '\\' && *p != '/';)
        if (*p == '.') {
            end = p;
            break;
        }
    if (ext)
        for (s = end; (*ext = *s++);)
            ext++;
    for (p = end; p > path;)
        if (*--p == '\\' || *p == '/') {
            p++;
            break;
        }
    if (name) {
        for (s = p; s < end;)
            *name++ = *s++;
        *name = '\0';
    }
    if (dir) {
        for (s = path; s < p;)
            *dir++ = *s++;
        *dir = '\0';
    }
}

int16_t *resampler(int16_t *data_in, size_t totalSampleCount, size_t in_sample_rate, size_t out_sample_rate) {
    if (data_in == nullptr)
        return nullptr;
    if (in_sample_rate == 0) return nullptr;
    size_t lengthIn = in_sample_rate / 100;
    size_t maxLen = out_sample_rate / 100;
    const int channels = 1;
    Resampler rs;
    if (rs.Reset(in_sample_rate, out_sample_rate, channels) == -1)
        return nullptr;
    size_t outLen = (size_t) (totalSampleCount * out_sample_rate / in_sample_rate);
    int16_t *data_out = (int16_t *) malloc(outLen * sizeof(int16_t));
    if (data_out == nullptr) return nullptr;
    size_t nCount = (totalSampleCount / lengthIn);
    size_t nLast = totalSampleCount - (lengthIn * nCount);
    int16_t *samplesIn = data_in;
    int16_t *samplesOut = data_out;

    outLen = 0;
    for (int i = 0; i < nCount; i++) {
        rs.Push(samplesIn, lengthIn, samplesOut, maxLen, outLen);
        samplesIn += lengthIn;
        samplesOut += outLen;
    }
    if (nLast != 0) {
        const int max_samples = 1920;
        int16_t samplePatchIn[max_samples] = {0};
        int16_t samplePatchOut[max_samples] = {0};
        memcpy(samplePatchIn, samplesIn, nLast * sizeof(int16_t));
        rs.Push(samplesIn, nLast, samplePatchOut, maxLen, outLen);
        memcpy(samplesOut, samplePatchOut, (nLast * out_sample_rate / in_sample_rate) * sizeof(int16_t));
    }
    return data_out;
}

void ResampleTo(char *in_file, char *out_file, size_t out_sample_rate = 16000) {
    //音频采样率
    uint32_t sampleRate = 0;
    //总音频采样数
    uint64_t inSampleCount = 0;
    int16_t *inBuffer = wavRead_int16(in_file, &sampleRate, &inSampleCount);
    //如果加载成功
    if (inBuffer != nullptr) {
        int16_t *outBuffer = resampler(inBuffer, (size_t) inSampleCount, sampleRate, out_sample_rate);
        if (outBuffer != nullptr) {
            size_t outSampleCount = (size_t) (inSampleCount * (out_sample_rate * 1.0f / sampleRate));
            wavWrite_int16(out_file, outBuffer, out_sample_rate, outSampleCount);
            free(outBuffer);
        }
        free(inBuffer);
    }
}

int main(int argc, char *argv[]) {
    printf("WebRtc Resampler\n");
    printf("博客:http://cpuimage.cnblogs.com/\n");
    printf("音频插值重采样\n");
    printf("支持采样率: 8k、16k、32k、48k、96k\n");
    if (argc < 2)
        return -1;
    char *in_file = argv[1];
    char drive[3];
    char dir[256];
    char fname[256];
    char ext[256];
    char out_file[1024];
    splitpath(in_file, drive, dir, fname, ext);
    sprintf(out_file, "%s%s%s_out%s", drive, dir, fname, ext);
    ResampleTo(in_file, out_file, 64000);
    printf("按任意键退出程序 \n");
    getchar();
    return 0;
}
