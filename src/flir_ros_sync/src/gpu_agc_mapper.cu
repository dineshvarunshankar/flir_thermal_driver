#include "flir_ros_sync/gpu_agc_mapper.h"
#include <cuda_runtime.h>
#include <cstdint>
#include <cstdio>
#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>

// Only compile CUDA code if CUDA is enabled
#if defined(WITH_CUDA) && WITH_CUDA

// -------- GPU AGC helpers (apply 16->8 LUT on GPU) --------
// These functions accelerate the mapping stage of the full AGC pipeline by applying
// a precomputed 65536-entry 8-bit LUT on the GPU.

__global__ void applyTransferKernel(const uint16_t* __restrict__ input,
                                    uint8_t* __restrict__ output,
                                    const uint8_t* __restrict__ lut,
                                    int total) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= total) return;
    uint16_t v = input[idx];
    output[idx] = lut[v];
}

// Simple global-atomics histogram. For 640x512 this is fine; can be optimized later with per-block histograms.
__global__ void histogramKernel(const uint16_t* __restrict__ input,
                                uint32_t* __restrict__ hist,
                                int total) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= total) return;
    atomicAdd(&hist[input[idx]], 1u);
}

static uint16_t* g_d_input = nullptr;
static uint8_t*  g_d_output = nullptr;
static uint8_t*  g_d_lut = nullptr;
static uint32_t* g_d_hist = nullptr;
static int g_capacity = 0; // number of pixels allocated
static cudaStream_t g_stream = nullptr;
static bool g_cuda_initialized = false;

// Helper function to reset GPU state
static bool resetGPUState() {
    fprintf(stderr, "[gpu_agc] Attempting to reset GPU state...\n");
    
    // First try to reset the current device
    cudaError_t cerr = cudaDeviceReset();
    if (cerr != cudaSuccess) {
        fprintf(stderr, "[gpu_agc] cudaDeviceReset failed: %s (%d)\n", cudaGetErrorString(cerr), (int)cerr);
        
        // If device reset fails, try to get last error and clear it
        cudaError_t last_error = cudaGetLastError();
        if (last_error != cudaSuccess) {
            fprintf(stderr, "[gpu_agc] Clearing last error: %s (%d)\n", cudaGetErrorString(last_error), (int)last_error);
        }
        
        // Try device reset one more time
        cerr = cudaDeviceReset();
        if (cerr != cudaSuccess) {
            fprintf(stderr, "[gpu_agc] Second cudaDeviceReset attempt failed: %s (%d)\n", cudaGetErrorString(cerr), (int)cerr);
            return false;
        }
    }
    
    fprintf(stderr, "[gpu_agc] GPU state reset successfully\n");
    return true;
}

// Helper function to validate device availability
static bool validateDevice(int device_id) {
    cudaError_t cerr;
    
    // Check if device exists and get its properties
    cudaDeviceProp device_prop;
    cerr = cudaGetDeviceProperties(&device_prop, device_id);
    if (cerr != cudaSuccess) {
        fprintf(stderr, "[gpu_agc] Failed to get device %d properties: %s (%d)\n",
                device_id, cudaGetErrorString(cerr), (int)cerr);
        return false;
    }
    
    // Check compute capability
    if (device_prop.major < 2) {
        fprintf(stderr, "[gpu_agc] Device %d compute capability %d.%d is too old (need 2.0+)\n",
                device_id, device_prop.major, device_prop.minor);
        return false;
    }
    
    // Check memory availability
    size_t free_memory, total_memory;
    cerr = cudaMemGetInfo(&free_memory, &total_memory);
    if (cerr != cudaSuccess) {
        fprintf(stderr, "[gpu_agc] Failed to get memory info for device %d: %s (%d)\n",
                device_id, cudaGetErrorString(cerr), (int)cerr);
        return false;
    }
    
    // Need at least 50MB free for our operations
    const size_t min_required_memory = 50 * 1024 * 1024;
    if (free_memory < min_required_memory) {
        fprintf(stderr, "[gpu_agc] Device %d has insufficient memory: %zu MB free, need %zu MB\n",
                device_id, free_memory / (1024 * 1024), min_required_memory / (1024 * 1024));
        return false;
    }
    
    fprintf(stderr, "[gpu_agc] Device %d (%s) validation passed: compute %d.%d, %zu/%zu MB memory\n",
            device_id, device_prop.name, device_prop.major, device_prop.minor,
            free_memory / (1024 * 1024), total_memory / (1024 * 1024));
    
    return true;
}

extern "C" bool cuda_agc_init(int width, int height) {
    int total = width * height;
    if (total <= 0) {
        fprintf(stderr, "[gpu_agc] Invalid image dimensions: %dx%d\n", width, height);
        return false;
    }

    // If already initialized for this size, just return success
    if (g_cuda_initialized && total <= g_capacity && g_d_input && g_d_output && g_d_lut) {
        return true;
    }

    // Clean up any existing resources first
    if (g_cuda_initialized) {
        cuda_agc_free();
    }

    fprintf(stderr, "[gpu_agc] Initializing CUDA AGC for %dx%d image...\n", width, height);

    
    // Check NVIDIA device files immediately
    fprintf(stderr, "[gpu_agc] === NVIDIA Device Check ===\n");
    const char* nvidia_devices[] = {"/dev/nvidia0", "/dev/nvidiactl", "/dev/nvidia-uvm", "/dev/nvidia-uvm-tools", "/dev/nvidia-modeset"};
    int found_devices = 0;
    for (int i = 0; i < 5; i++) {
        if (access(nvidia_devices[i], F_OK) == 0) {
            found_devices++;
            bool readable = (access(nvidia_devices[i], R_OK) == 0);
            bool writable = (access(nvidia_devices[i], W_OK) == 0);
            fprintf(stderr, "[gpu_agc] %s: EXISTS (r:%s w:%s)\n", nvidia_devices[i],
                    readable ? "yes" : "no", writable ? "yes" : "no");
        } else {
            fprintf(stderr, "[gpu_agc] %s: MISSING\n", nvidia_devices[i]);
        }
    }
    fprintf(stderr, "[gpu_agc] NVIDIA devices found: %d/5\n", found_devices);
    
    // Test CUDA versions before device enumeration
    int runtime_version = 0;
    cudaError_t rt_err = cudaRuntimeGetVersion(&runtime_version);
    if (rt_err == cudaSuccess) {
        fprintf(stderr, "[gpu_agc] CUDA Runtime: %d.%d\n", runtime_version/1000, (runtime_version%100)/10);
    } else {
        fprintf(stderr, "[gpu_agc] CUDA Runtime check FAILED: %s\n", cudaGetErrorString(rt_err));
    }
    
    // Check for critical CUDA driver library
    void* cuda_driver = dlopen("libcuda.so.1", RTLD_LAZY);
    if (cuda_driver) {
        fprintf(stderr, "[gpu_agc] CUDA Driver Library (libcuda.so.1): AVAILABLE\n");
        dlclose(cuda_driver);
    } else {
        fprintf(stderr, "[gpu_agc] CUDA Driver Library (libcuda.so.1): MISSING - %s\n", dlerror());
        fprintf(stderr, "[gpu_agc] *** THIS IS LIKELY THE CAUSE OF ERROR 999 ***\n");
    }
    
    // Check for NVIDIA ML library too
    void* nvidia_ml = dlopen("libnvidia-ml.so.1", RTLD_LAZY);
    if (nvidia_ml) {
        fprintf(stderr, "[gpu_agc] NVIDIA ML Library: AVAILABLE\n");
        dlclose(nvidia_ml);
    } else {
        fprintf(stderr, "[gpu_agc] NVIDIA ML Library: MISSING - %s\n", dlerror());
    }
    
    // Check CUDA driver version and compare with runtime
    int driver_version = 0;
    cudaError_t drv_err = cudaDriverGetVersion(&driver_version);
    if (drv_err == cudaSuccess) {
        fprintf(stderr, "[gpu_agc] CUDA Driver Version: %d.%d\n", driver_version/1000, (driver_version%100)/10);
        
        // Check for version compatibility
        if (driver_version < runtime_version) {
            fprintf(stderr, "[gpu_agc] *** VERSION MISMATCH: Driver (%d.%d) < Runtime (%d.%d) ***\n",
                    driver_version/1000, (driver_version%100)/10,
                    runtime_version/1000, (runtime_version%100)/10);
            fprintf(stderr, "[gpu_agc] *** THIS IS THE CAUSE OF ERROR 999 ***\n");
            fprintf(stderr, "[gpu_agc] Solution: Update NVIDIA driver to >= %d.%d\n",
                    runtime_version/1000, (runtime_version%100)/10);
        } else {
            fprintf(stderr, "[gpu_agc] Driver/Runtime versions compatible\n");
        }
    } else {
        fprintf(stderr, "[gpu_agc] CUDA Driver version check FAILED: %s\n", cudaGetErrorString(drv_err));
    }
    
    // Do check for CUDA_VISIBLE_DEVICES environment variable
    char* cuda_visible_devices = getenv("CUDA_VISIBLE_DEVICES");
    if (cuda_visible_devices) {
        fprintf(stderr, "[gpu_agc] CUDA_VISIBLE_DEVICES: %s\n", cuda_visible_devices);
    } else {
        fprintf(stderr, "[gpu_agc] CUDA_VISIBLE_DEVICES: NOT SET\n");
    }
    
    // Process-level diagnostics before attempting cudaGetDeviceCount
    fprintf(stderr, "[gpu_agc] === Process-Level GPU Access Check ===\n");
    
    // Check user groups (video/render groups often required for GPU access)
    fprintf(stderr, "[gpu_agc] User ID: %d, Group ID: %d\n", getuid(), getgid());
    
    // Try basic CUDA initialization before device count
    fprintf(stderr, "[gpu_agc] Testing basic CUDA initialization...\n");
    
    // Try to initialize CUDA runtime
    cudaError_t init_err = cudaFree(0);  // Minimal CUDA operation
    if (init_err == cudaSuccess) {
        fprintf(stderr, "[gpu_agc] Basic CUDA initialization: SUCCESS\n");
    } else {
        fprintf(stderr, "[gpu_agc] Basic CUDA initialization FAILED: %s (%d)\n",
                cudaGetErrorString(init_err), (int)init_err);
        if (init_err == cudaErrorUnknown) {
            fprintf(stderr, "[gpu_agc] *** CUDA initialization fails with error 999 ***\n");
            fprintf(stderr, "[gpu_agc] This suggests GPU hardware or driver crash\n");
        }
    }
    
    // Clear any errors before proceeding
    cudaGetLastError();

    cudaError_t cerr;
    
    // Step 1: Check if CUDA is available at all
    int device_count = 0;
    cerr = cudaGetDeviceCount(&device_count);
    if (cerr != cudaSuccess) {
        fprintf(stderr, "[gpu_agc] cudaGetDeviceCount failed: %s (%d)\n", cudaGetErrorString(cerr), (int)cerr);
        
        // Error 999 suggests fundamental access issues
        if (cerr == cudaErrorUnknown) {
            fprintf(stderr, "[gpu_agc] === CUDA Error 999 Troubleshooting ===\n");
            fprintf(stderr, "[gpu_agc] This usually indicates:\n");
            fprintf(stderr, "[gpu_agc] 1. CUDA driver/runtime version mismatch\n");
            fprintf(stderr, "[gpu_agc] 2. GPU permissions issue (check /dev/nvidia*)\n");
            fprintf(stderr, "[gpu_agc] 3. Container GPU access not properly configured\n");
            fprintf(stderr, "[gpu_agc] 4. NVIDIA driver not loaded or crashed\n");
            fprintf(stderr, "[gpu_agc] Try: nvidia-smi, docker run --gpus all, or restart nvidia drivers\n");
            fprintf(stderr, "[gpu_agc] =========================================\n");
        }
        
        // Try to reset and check again
        if (resetGPUState()) {
            cerr = cudaGetDeviceCount(&device_count);
            if (cerr != cudaSuccess) {
                fprintf(stderr, "[gpu_agc] cudaGetDeviceCount failed after reset: %s (%d)\n", cudaGetErrorString(cerr), (int)cerr);
                return false;
            }
        } else {
            return false;
        }
    }
    
    if (device_count == 0) {
        fprintf(stderr, "[gpu_agc] No CUDA devices available (count: %d)\n", device_count);
        return false;
    }
    
    fprintf(stderr, "[gpu_agc] Found %d CUDA device(s)\n", device_count);
    
    // Step 2: Try to find a suitable device
    int selected_device = -1;
    for (int dev = 0; dev < device_count; dev++) {
        fprintf(stderr, "[gpu_agc] Checking device %d...\n", dev);
        
        // Try to set this device
        cerr = cudaSetDevice(dev);
        if (cerr != cudaSuccess) {
            fprintf(stderr, "[gpu_agc] Failed to set device %d: %s (%d)\n", dev, cudaGetErrorString(cerr), (int)cerr);
            
            // If setting device fails with unknown error, try to reset and retry
            if (cerr == cudaErrorUnknown) {
                fprintf(stderr, "[gpu_agc] Unknown error setting device %d, attempting reset...\n", dev);
                if (resetGPUState()) {
                    cerr = cudaSetDevice(dev);
                    if (cerr == cudaSuccess) {
                        fprintf(stderr, "[gpu_agc] Device %d set successfully after reset\n", dev);
                    } else {
                        fprintf(stderr, "[gpu_agc] Device %d still failed after reset: %s (%d)\n", dev, cudaGetErrorString(cerr), (int)cerr);
                        continue;
                    }
                } else {
                    continue;
                }
            } else {
                continue;
            }
        }
        
        // Validate this device
        if (validateDevice(dev)) {
            selected_device = dev;
            fprintf(stderr, "[gpu_agc] Selected device %d\n", dev);
            break;
        } else {
            fprintf(stderr, "[gpu_agc] Device %d failed validation\n", dev);
        }
    }
    
    if (selected_device < 0) {
        fprintf(stderr, "[gpu_agc] No suitable CUDA device found\n");
        return false;
    }
    
    // Step 3: Initialize CUDA context with a simple operation
    cerr = cudaSetDevice(selected_device);
    if (cerr != cudaSuccess) {
        fprintf(stderr, "[gpu_agc] Failed to set selected device %d: %s (%d)\n", selected_device, cudaGetErrorString(cerr), (int)cerr);
        return false;
    }
    
    // Step 4: Create stream
    cerr = cudaStreamCreate(&g_stream);
    if (cerr != cudaSuccess) {
        fprintf(stderr, "[gpu_agc] cudaStreamCreate failed: %s (%d)\n", cudaGetErrorString(cerr), (int)cerr);
        return false;
    }
    
    // Step 5: Test basic CUDA functionality
    void* test_ptr = nullptr;
    cerr = cudaMalloc(&test_ptr, 1024);
    if (cerr != cudaSuccess) {
        fprintf(stderr, "[gpu_agc] Test cudaMalloc failed: %s (%d)\n", cudaGetErrorString(cerr), (int)cerr);
        cudaStreamDestroy(g_stream);
        g_stream = nullptr;
        return false;
    }
    cudaFree(test_ptr);
    
    // Step 6: Allocate buffers
    size_t in_bytes = static_cast<size_t>(total) * sizeof(uint16_t);
    size_t out_bytes = static_cast<size_t>(total) * sizeof(uint8_t);
    
    cerr = cudaMalloc(&g_d_input, in_bytes);
    if (cerr != cudaSuccess) {
        fprintf(stderr, "[gpu_agc] cudaMalloc(g_d_input, %zu bytes) failed: %s (%d)\n", in_bytes, cudaGetErrorString(cerr), (int)cerr);
        cudaStreamDestroy(g_stream);
        g_stream = nullptr;
        return false;
    }
    
    cerr = cudaMalloc(&g_d_output, out_bytes);
    if (cerr != cudaSuccess) {
        fprintf(stderr, "[gpu_agc] cudaMalloc(g_d_output, %zu bytes) failed: %s (%d)\n", out_bytes, cudaGetErrorString(cerr), (int)cerr);
        cudaFree(g_d_input);
        g_d_input = nullptr;
        cudaStreamDestroy(g_stream);
        g_stream = nullptr;
        return false;
    }
    
    // 65536-entry 8-bit LUT
    cerr = cudaMalloc(&g_d_lut, 65536 * sizeof(uint8_t));
    if (cerr != cudaSuccess) {
        fprintf(stderr, "[gpu_agc] cudaMalloc(g_d_lut, %zu bytes) failed: %s (%d)\n", 65536 * sizeof(uint8_t), cudaGetErrorString(cerr), (int)cerr);
        cudaFree(g_d_input);
        cudaFree(g_d_output);
        g_d_input = nullptr;
        g_d_output = nullptr;
        cudaStreamDestroy(g_stream);
        g_stream = nullptr;
        return false;
    }
    
    // 65536-entry histogram (uint32)
    cerr = cudaMalloc(&g_d_hist, 65536 * sizeof(uint32_t));
    if (cerr != cudaSuccess) {
        fprintf(stderr, "[gpu_agc] cudaMalloc(g_d_hist, %zu bytes) failed: %s (%d)\n", 65536 * sizeof(uint32_t), cudaGetErrorString(cerr), (int)cerr);
        cudaFree(g_d_input);
        cudaFree(g_d_output);
        cudaFree(g_d_lut);
        g_d_input = nullptr;
        g_d_output = nullptr;
        g_d_lut = nullptr;
        cudaStreamDestroy(g_stream);
        g_stream = nullptr;
        return false;
    }
    
    g_capacity = total;
    g_cuda_initialized = true;
    
    fprintf(stderr, "[gpu_agc] CUDA AGC initialization successful on device %d\n", selected_device);
    return true;
}

extern "C" bool cuda_agc_apply(const uint16_t* h_input,
                                uint8_t* h_output,
                                const uint8_t* h_lut,
                                int width,
                                int height) {
    int total = width * height;
    if (total <= 0) return false;
    if (!g_cuda_initialized || !g_d_input || !g_d_output || !g_d_lut || !g_stream) {
        fprintf(stderr, "[gpu_agc] CUDA not properly initialized for apply operation\n");
        return false;
    }
    
    size_t in_bytes = static_cast<size_t>(total) * sizeof(uint16_t);
    size_t out_bytes = static_cast<size_t>(total) * sizeof(uint8_t);
    
    cudaError_t cerr;
    
    // copy LUT (small, but copy every call to reflect updates)
    cerr = cudaMemcpyAsync(g_d_lut, h_lut, 65536 * sizeof(uint8_t), cudaMemcpyHostToDevice, g_stream);
    if (cerr != cudaSuccess) {
        fprintf(stderr, "[gpu_agc] cudaMemcpyAsync(lut) failed: %s (%d)\n", cudaGetErrorString(cerr), (int)cerr);
        return false;
    }
    
    cerr = cudaMemcpyAsync(g_d_input, h_input, in_bytes, cudaMemcpyHostToDevice, g_stream);
    if (cerr != cudaSuccess) {
        fprintf(stderr, "[gpu_agc] cudaMemcpyAsync(input) failed: %s (%d)\n", cudaGetErrorString(cerr), (int)cerr);
        return false;
    }
    
    int block = 256;
    int grid = (total + block - 1) / block;
    applyTransferKernel<<<grid, block, 0, g_stream>>>(g_d_input, g_d_output, g_d_lut, total);
    
    cerr = cudaGetLastError();
    if (cerr != cudaSuccess) {
        fprintf(stderr, "[gpu_agc] kernel launch failed: %s (%d)\n", cudaGetErrorString(cerr), (int)cerr);
        return false;
    }
    
    cerr = cudaMemcpyAsync(h_output, g_d_output, out_bytes, cudaMemcpyDeviceToHost, g_stream);
    if (cerr != cudaSuccess) {
        fprintf(stderr, "[gpu_agc] cudaMemcpyAsync(output) failed: %s (%d)\n", cudaGetErrorString(cerr), (int)cerr);
        return false;
    }
    
    cerr = cudaStreamSynchronize(g_stream);
    if (cerr != cudaSuccess) {
        fprintf(stderr, "[gpu_agc] cudaStreamSynchronize failed: %s (%d)\n", cudaGetErrorString(cerr), (int)cerr);
        return false;
    }
    
    return true;
}

extern "C" bool cuda_agc_histogram(const uint16_t* h_input,
                                    int width,
                                    int height,
                                    uint32_t* h_hist) {
    int total = width * height;
    if (total <= 0) return false;
    if (!g_cuda_initialized || !g_d_input || !g_d_hist || !g_stream) {
        fprintf(stderr, "[gpu_agc] CUDA not properly initialized for histogram operation\n");
        return false;
    }
    
    size_t in_bytes = static_cast<size_t>(total) * sizeof(uint16_t);
    
    cudaError_t cerr;
    
    // upload input
    cerr = cudaMemcpyAsync(g_d_input, h_input, in_bytes, cudaMemcpyHostToDevice, g_stream);
    if (cerr != cudaSuccess) {
        fprintf(stderr, "[gpu_agc] cudaMemcpyAsync(input) failed: %s (%d)\n", cudaGetErrorString(cerr), (int)cerr);
        return false;
    }
    
    // zero histogram
    cerr = cudaMemsetAsync(g_d_hist, 0, 65536 * sizeof(uint32_t), g_stream);
    if (cerr != cudaSuccess) {
        fprintf(stderr, "[gpu_agc] cudaMemsetAsync(hist) failed: %s (%d)\n", cudaGetErrorString(cerr), (int)cerr);
        return false;
    }
    
    // launch kernel
    int block = 256;
    int grid = (total + block - 1) / block;
    histogramKernel<<<grid, block, 0, g_stream>>>(g_d_input, g_d_hist, total);
    
    cerr = cudaGetLastError();
    if (cerr != cudaSuccess) {
        fprintf(stderr, "[gpu_agc] histogram kernel failed: %s (%d)\n", cudaGetErrorString(cerr), (int)cerr);
        return false;
    }
    
    // copy back
    cerr = cudaMemcpyAsync(h_hist, g_d_hist, 65536 * sizeof(uint32_t), cudaMemcpyDeviceToHost, g_stream);
    if (cerr != cudaSuccess) {
        fprintf(stderr, "[gpu_agc] cudaMemcpyAsync(hist->host) failed: %s (%d)\n", cudaGetErrorString(cerr), (int)cerr);
        return false;
    }
    
    cerr = cudaStreamSynchronize(g_stream);
    if (cerr != cudaSuccess) {
        fprintf(stderr, "[gpu_agc] cudaStreamSynchronize failed: %s (%d)\n", cudaGetErrorString(cerr), (int)cerr);
        return false;
    }
    
    return true;
}

extern "C" void cuda_agc_free() {
    fprintf(stderr, "[gpu_agc] Cleaning up CUDA resources...\n");
    
    if (g_d_input) {
        cudaFree(g_d_input);
        g_d_input = nullptr;
    }
    if (g_d_output) {
        cudaFree(g_d_output);
        g_d_output = nullptr;
    }
    if (g_d_lut) {
        cudaFree(g_d_lut);
        g_d_lut = nullptr;
    }
    if (g_d_hist) {
        cudaFree(g_d_hist);
        g_d_hist = nullptr;
    }
    if (g_stream) {
        cudaStreamDestroy(g_stream);
        g_stream = nullptr;
    }
    
    g_capacity = 0;
    g_cuda_initialized = false;
    
    fprintf(stderr, "[gpu_agc] CUDA resources cleaned up\n");
}

#else // WITH_CUDA not defined or 0

// Provide stub implementations for when CUDA is not available
extern "C" bool cuda_agc_init(int width, int height) {
    (void)width; (void)height;
    fprintf(stderr, "[gpu_agc] CUDA support not compiled in\n");
    return false;
}

extern "C" bool cuda_agc_apply(const uint16_t* h_input,
                                uint8_t* h_output,
                                const uint8_t* h_lut,
                                int width,
                                int height) {
    (void)h_input; (void)h_output; (void)h_lut; (void)width; (void)height;
    return false;
}

extern "C" bool cuda_agc_histogram(const uint16_t* h_input,
                                    int width,
                                    int height,
                                    uint32_t* h_hist) {
    (void)h_input; (void)width; (void)height; (void)h_hist;
    return false;
}

extern "C" void cuda_agc_free() {
    // no-op
}

#endif // WITH_CUDA
