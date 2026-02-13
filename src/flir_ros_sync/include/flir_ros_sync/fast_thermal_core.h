#ifndef FAST_THERMAL_CORE_H
#define FAST_THERMAL_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

// Fast histogram computation using optimized memory access
void compute_histogram_fast(const uint16_t* image_data, size_t pixel_count,
                           uint32_t* histogram, int num_threads);

// Fast cumulative histogram with prefetching
void compute_cumulative_fast(const uint32_t* histogram, uint64_t* cumulative);

// Fast percentile calculation using partial sort
void find_percentiles_fast(const uint16_t* data, size_t count,
                          float low_percent, float high_percent,
                          uint16_t* low_val, uint16_t* high_val);

// Fast transfer function application with SIMD
void apply_transfer_fast(const uint16_t* input, uint8_t* output,
                        const uint8_t* lut, size_t pixel_count, int num_threads);

// Fast tail clipping with in-place operation
void clip_tails_fast(uint16_t* data, size_t count,
                    uint16_t low_threshold, uint16_t high_threshold,
                    int num_threads);

// Fast Gaussian blur approximation using box filter
void fast_gaussian_blur(const float* input, float* output,
                       int width, int height, int radius);

// Fast min-max calculation with SIMD
void find_min_max_fast(const uint16_t* data, size_t count,
                      uint16_t* min_val, uint16_t* max_val);

#ifdef __cplusplus
}
#endif

#endif // FAST_THERMAL_CORE_H