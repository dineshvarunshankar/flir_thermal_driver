#include "flir_ros_sync/fast_thermal_core.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#ifdef __SSE2__
#include <emmintrin.h>
#ifdef __SSE4_1__
#include <smmintrin.h>
#endif
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

// SSE2-compatible helpers for unsigned 16-bit min/max when SSE4.1 is unavailable
#ifdef __SSE2__
#ifndef __SSE4_1__
static inline __m128i mm_min_epu16_sse2(__m128i a, __m128i b) {
    const __m128i bias = _mm_set1_epi16((short)0x8000);
    __m128i ax = _mm_xor_si128(a, bias);
    __m128i bx = _mm_xor_si128(b, bias);
    __m128i m = _mm_min_epi16(ax, bx);
    return _mm_xor_si128(m, bias);
}

static inline __m128i mm_max_epu16_sse2(__m128i a, __m128i b) {
    const __m128i bias = _mm_set1_epi16((short)0x8000);
    __m128i ax = _mm_xor_si128(a, bias);
    __m128i bx = _mm_xor_si128(b, bias);
    __m128i m = _mm_max_epi16(ax, bx);
    return _mm_xor_si128(m, bias);
}
#endif // !__SSE4_1__
#endif // __SSE2__

// Fast histogram using cache-friendly access pattern
void compute_histogram_fast(const uint16_t* image_data, size_t pixel_count,
                           uint32_t* histogram, int num_threads) {
    // Clear histogram
    memset(histogram, 0, 65536 * sizeof(uint32_t));
    
#ifdef _OPENMP
    if (num_threads > 1) {
        // Create per-thread histograms to avoid cache contention
        uint32_t** thread_hists = (uint32_t**)calloc(num_threads, sizeof(uint32_t*));
        for (int i = 0; i < num_threads; i++) {
            thread_hists[i] = (uint32_t*)calloc(65536, sizeof(uint32_t));
        }
        
        #pragma omp parallel num_threads(num_threads)
        {
            int tid = omp_get_thread_num();
            uint32_t* local_hist = thread_hists[tid];
            
            #pragma omp for schedule(static)
            for (size_t i = 0; i < pixel_count; i++) {
                local_hist[image_data[i]]++;
            }
        }
        
        // Merge thread-local histograms
        #pragma omp parallel for num_threads(num_threads)
        for (int i = 0; i < 65536; i++) {
            uint32_t sum = 0;
            for (int t = 0; t < num_threads; t++) {
                sum += thread_hists[t][i];
            }
            histogram[i] = sum;
        }
        
        // Cleanup
        for (int i = 0; i < num_threads; i++) {
            free(thread_hists[i]);
        }
        free(thread_hists);
    } else
#endif
    {
        // Single-threaded version with loop unrolling
        size_t i = 0;
        for (; i + 8 <= pixel_count; i += 8) {
            histogram[image_data[i]]++;
            histogram[image_data[i+1]]++;
            histogram[image_data[i+2]]++;
            histogram[image_data[i+3]]++;
            histogram[image_data[i+4]]++;
            histogram[image_data[i+5]]++;
            histogram[image_data[i+6]]++;
            histogram[image_data[i+7]]++;
        }
        for (; i < pixel_count; i++) {
            histogram[image_data[i]]++;
        }
    }
}

// Fast cumulative histogram
void compute_cumulative_fast(const uint32_t* histogram, uint64_t* cumulative) {
    cumulative[0] = histogram[0];
    
    // Unroll loop for better performance
    size_t i = 1;
    for (; i + 4 <= 65536; i += 4) {
        cumulative[i] = cumulative[i-1] + histogram[i];
        cumulative[i+1] = cumulative[i] + histogram[i+1];
        cumulative[i+2] = cumulative[i+1] + histogram[i+2];
        cumulative[i+3] = cumulative[i+2] + histogram[i+3];
    }
    for (; i < 65536; i++) {
        cumulative[i] = cumulative[i-1] + histogram[i];
    }
}

// Comparison function for qsort
static int compare_uint16(const void* a, const void* b) {
    return (*(uint16_t*)a - *(uint16_t*)b);
}

// Fast percentile using partial sort (much faster than full sort)
void find_percentiles_fast(const uint16_t* data, size_t count,
                          float low_percent, float high_percent,
                          uint16_t* low_val, uint16_t* high_val) {
    if (count == 0) {
        *low_val = 0;
        *high_val = 65535;
        return;
    }
    
    size_t low_idx = (size_t)(count * low_percent / 100.0f);
    size_t high_idx = (size_t)(count * high_percent / 100.0f);
    
    // Clamp indices
    if (low_idx >= count) low_idx = count - 1;
    if (high_idx >= count) high_idx = count - 1;
    
    // For small arrays or when we need both ends, just sort
    if (count < 10000) {
        uint16_t* temp = (uint16_t*)malloc(count * sizeof(uint16_t));
        memcpy(temp, data, count * sizeof(uint16_t));
        qsort(temp, count, sizeof(uint16_t), compare_uint16);
        *low_val = temp[low_idx];
        *high_val = temp[high_idx];
        free(temp);
    } else {
        // Use nth_element approach for large arrays
        // Sample the data for approximate percentiles (much faster)
        size_t sample_size = 10000;
        uint16_t* sample = (uint16_t*)malloc(sample_size * sizeof(uint16_t));
        size_t step = count / sample_size;
        
        for (size_t i = 0; i < sample_size; i++) {
            sample[i] = data[i * step];
        }
        
        qsort(sample, sample_size, sizeof(uint16_t), compare_uint16);
        
        size_t sample_low_idx = (size_t)(sample_size * low_percent / 100.0f);
        size_t sample_high_idx = (size_t)(sample_size * high_percent / 100.0f);
        
        *low_val = sample[sample_low_idx];
        *high_val = sample[sample_high_idx];
        
        free(sample);
    }
}

// Fast LUT application with OpenMP and SIMD
void apply_transfer_fast(const uint16_t* input, uint8_t* output,
                        const uint8_t* lut, size_t pixel_count, int num_threads) {
#ifdef _OPENMP
    #pragma omp parallel for num_threads(num_threads) schedule(static, 4096)
#endif
    for (size_t i = 0; i < pixel_count; i++) {
        output[i] = lut[input[i]];
    }
}

// Fast tail clipping
void clip_tails_fast(uint16_t* data, size_t count,
                    uint16_t low_threshold, uint16_t high_threshold,
                    int num_threads) {
#ifdef _OPENMP
    #pragma omp parallel for num_threads(num_threads) schedule(static, 4096)
#endif
    for (size_t i = 0; i < count; i++) {
        uint16_t val = data[i];
        if (val < low_threshold) {
            data[i] = low_threshold;
        } else if (val > high_threshold) {
            data[i] = high_threshold;
        }
    }
}

// Fast box filter (approximates Gaussian)
void fast_gaussian_blur(const float* input, float* output,
                       int width, int height, int radius) {
    // Simple box filter - much faster than true Gaussian
    // First pass: horizontal
    float* temp = (float*)malloc(width * height * sizeof(float));
    
    #pragma omp parallel for
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float sum = 0;
            int count = 0;
            
            for (int dx = -radius; dx <= radius; dx++) {
                int nx = x + dx;
                if (nx >= 0 && nx < width) {
                    sum += input[y * width + nx];
                    count++;
                }
            }
            
            temp[y * width + x] = sum / count;
        }
    }
    
    // Second pass: vertical
    #pragma omp parallel for
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float sum = 0;
            int count = 0;
            
            for (int dy = -radius; dy <= radius; dy++) {
                int ny = y + dy;
                if (ny >= 0 && ny < height) {
                    sum += temp[ny * width + x];
                    count++;
                }
            }
            
            output[y * width + x] = sum / count;
        }
    }
    
    free(temp);
}

// Fast min-max with SIMD
void find_min_max_fast(const uint16_t* data, size_t count,
                      uint16_t* min_val, uint16_t* max_val) {
    if (count == 0) {
        *min_val = 0;
        *max_val = 65535;
        return;
    }
    
    uint16_t min_v = 65535;
    uint16_t max_v = 0;
    
#ifdef __SSE2__
    if (count >= 8) {
        // Initialize from first vector to avoid constant overflow and ensure correctness
        size_t i = 0;
        __m128i vmin = _mm_loadu_si128((const __m128i*)(data));
        __m128i vmax = vmin;
        
        i += 8;
        for (; i + 8 <= count; i += 8) {
            __m128i vals = _mm_loadu_si128((__m128i*)(data + i));
            #ifdef __SSE4_1__
                vmin = _mm_min_epu16(vmin, vals);
                vmax = _mm_max_epu16(vmax, vals);
            #else
                vmin = mm_min_epu16_sse2(vmin, vals);
                vmax = mm_max_epu16_sse2(vmax, vals);
            #endif
        }
        
        // Extract min/max from vectors
        uint16_t min_arr[8], max_arr[8];
        _mm_storeu_si128((__m128i*)min_arr, vmin);
        _mm_storeu_si128((__m128i*)max_arr, vmax);
        
        for (int j = 0; j < 8; j++) {
            if (min_arr[j] < min_v) min_v = min_arr[j];
            if (max_arr[j] > max_v) max_v = max_arr[j];
        }
        
        // Handle remaining elements
        for (; i < count; i++) {
            if (data[i] < min_v) min_v = data[i];
            if (data[i] > max_v) max_v = data[i];
        }
    } else
#endif
    {
        // Fallback to simple loop
        for (size_t i = 0; i < count; i++) {
            if (data[i] < min_v) min_v = data[i];
            if (data[i] > max_v) max_v = data[i];
        }
    }
    
    *min_val = min_v;
    *max_val = max_v;
}