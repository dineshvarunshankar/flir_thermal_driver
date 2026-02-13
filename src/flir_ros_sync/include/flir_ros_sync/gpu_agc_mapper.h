// New GPU AGC mapper helper C API
#ifndef GPU_AGC_MAPPER_H
#define GPU_AGC_MAPPER_H

#include <cstddef>
#include <cstdint>

// These helpers provide a simple C interface to the CUDA kernels that
// apply a precomputed 8-bit LUT to a 16-bit thermal frame on the GPU.
// The full AGC pipeline (histogram analysis, bounds, LUT build) runs on CPU;
// only the final 16u->8u mapping is accelerated here.

#ifdef __cplusplus
extern "C" {
#endif

// Initialize any persistent CUDA resources (device buffers, etc.) as needed
// for the given image width/height. Returns true on success.
bool cuda_agc_init(int width, int height);

// Apply the 8-bit LUT to the 16-bit host image buffer, writing to an 8-bit
// host output buffer. Internally uses CUDA to accelerate the mapping.
bool cuda_agc_apply(const std::uint16_t* h_input_16u,
                    std::uint8_t* h_output_8u,
                    const std::uint8_t* h_lut_8u,
                    int width,
                    int height);

// Compute 65536-bin histogram on the GPU and return it to host buffer.
// h_hist must point to an array of 65536 uint32_t elements.
bool cuda_agc_histogram(const std::uint16_t* h_input_16u,
                        int width,
                        int height,
                        std::uint32_t* h_hist_65536);

// Free any persistent CUDA resources allocated by cuda_agc_init.
void cuda_agc_free();

#ifdef __cplusplus
} // extern "C"
#endif

#endif // GPU_AGC_MAPPER_H
