#include "flir_ros_sync/thermal_processor.h"
#include <rclcpp_components/register_node_macro.hpp>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>
#include <cstring>
#include <chrono>

#ifdef WITH_OPENMP
#include <omp.h>
#endif

namespace flir_ros_sync {

// Remove all timing for production
#define TIME_BLOCK(name, code) code

extern "C" {
#include "flir_ros_sync/fast_thermal_core.h"
}

// Forward declarations for GPU AGC helpers implemented in gpu_agc_mapper.cu
#ifdef WITH_CUDA
extern "C" {
bool cuda_agc_init(int width, int height);
bool cuda_agc_apply(const uint16_t* h_input, uint8_t* h_output, const uint8_t* h_lut, int width, int height);
void cuda_agc_free();
}
#endif

ThermalProcessor::ThermalProcessor(const Config& config) : config_(config) {
    // Initialize transfer functions
    transfer_function_.fill(0.0f);
    previous_transfer_.fill(0.0f);
    
    // Pre-allocate working buffers
    histogram_buffer_.reset(new uint32_t[HISTOGRAM_BINS]);
    cumulative_buffer_.reset(new uint64_t[HISTOGRAM_BINS]);
    lut_buffer_.reset(new uint8_t[HISTOGRAM_BINS]);
    
    if (!config_.K.empty() && !config_.distortion.empty()) {
        initializeRectificationMaps();
    }
    
#ifdef WITH_OPENMP
    num_threads_ = omp_get_max_threads();
    RCLCPP_INFO(rclcpp::get_logger("thermal_processor"),
                "OpenMP enabled with %d threads", num_threads_);
#else
    num_threads_ = 1;
#endif

#ifdef WITH_CUDA
    // Defer actual allocation until we see the first image (to know size)
    use_cuda_ = false;
#endif
}

ThermalProcessor::~ThermalProcessor() {
    cleanupCudaResources();
}

void ThermalProcessor::updateConfig(const Config& config) {
    config_ = config;
    if (!config_.K.empty() && !config_.distortion.empty()) {
        initializeRectificationMaps();
    }
}

void ThermalProcessor::updateCalibration(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) {
    // Convert ROS camera_info to OpenCV format
    config_.K = cv::Mat(3, 3, CV_64F, const_cast<double*>(camera_info->k.data())).clone();
    config_.distortion = cv::Mat(1, camera_info->d.size(), CV_64F, const_cast<double*>(camera_info->d.data())).clone();
    
    initializeRectificationMaps();
}

void ThermalProcessor::initializeRectificationMaps() {
    if (config_.K.empty() || config_.distortion.empty()) {
        return;
    }
    
    // Assume image size from typical FLIR camera dimensions
    cv::Size image_size(640, 512);
    
    cv::initUndistortRectifyMap(
        config_.K, config_.distortion, cv::Mat(), config_.K,
        image_size, CV_16SC2, map1_, map2_
    );
    
    maps_initialized_ = true;
}

cv::Mat ThermalProcessor::applyFlirAGC(const cv::Mat& image_16bit) {
    CV_Assert(image_16bit.type() == CV_16UC1);
    auto t_start = std::chrono::steady_clock::now();
    RCLCPP_DEBUG_ONCE(rclcpp::get_logger("thermal_processor"),
                 "applyFlirAGC enter: process=%s tail=%.2f max_gain=%.2f gamma=%.2f linear_percent=%.1f plateau=%.1f damping=%.1f palette=%d enhance=%d",
                 config_.process.c_str(), config_.tail_rejection, config_.max_gain, config_.gamma,
                 config_.linear_percent, config_.plateau, config_.damping_factor, config_.color_palette,
                 static_cast<int>(config_.enhance));
    
#ifdef WITH_CUDA
    // Try GPU path: lazily initialize on first frame and use for larger images
    if (image_16bit.rows * image_16bit.cols > 100000) {
        if (!use_cuda_) {
            initializeCudaResources(static_cast<size_t>(image_16bit.cols), static_cast<size_t>(image_16bit.rows));
        }
        if (use_cuda_) {
            cv::Mat gpu_out = applyFlirAGCCuda(image_16bit);
            auto t_end = std::chrono::steady_clock::now();
            double ms = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count() / 1000.0;
            RCLCPP_DEBUG_ONCE(rclcpp::get_logger("thermal_processor"),
                         "Using CUDA path (%dx%d) in %.2f ms",
                         image_16bit.cols, image_16bit.rows, ms);
            return gpu_out;
        }
    }
#endif
    
    // Step 1: Apply tail rejection to exclude outliers
    cv::Mat image_clipped;
    TIME_BLOCK("tail_rejection", {
        image_clipped = applyTailRejection(image_16bit, config_.tail_rejection);
    });
    
    // Step 2: Build stats view (ROI or full) and generate transfer function
    cv::Mat stats_view = image_clipped;
    if (config_.roi_enable) {
        int x = std::max(0, config_.roi_x);
        int y = std::max(0, config_.roi_y);
        int w = config_.roi_width > 0 ? config_.roi_width : image_clipped.cols;
        int h = config_.roi_height > 0 ? config_.roi_height : image_clipped.rows;
        if (x + w > image_clipped.cols) w = image_clipped.cols - x;
        if (y + h > image_clipped.rows) h = image_clipped.rows - y;
        if (w > 0 && h > 0) {
            stats_view = image_clipped(cv::Rect(x, y, w, h));
            RCLCPP_DEBUG_ONCE(rclcpp::get_logger("thermal_processor"),
                              "Software AGC ROI active: x=%d y=%d w=%d h=%d", x, y, w, h);
        }
    }
    std::array<float, HISTOGRAM_BINS> histogram_transfer;
    TIME_BLOCK("transfer_function", {
        if (config_.process == "information_based") {
            histogram_transfer = informationBasedEqualization(stats_view);
        } else {
            histogram_transfer = plateauEqualization(stats_view);
        }
    });
    
    // Step 3: Create linear mapping function (based on ROI stats view)
    std::array<float, HISTOGRAM_BINS> linear_transfer;
    TIME_BLOCK("linear_transfer", {
        linear_transfer = createLinearTransfer(stats_view);
    });
    
    // Step 4: Blend histogram-based and linear transfers
    std::array<float, HISTOGRAM_BINS> blended_transfer;
    TIME_BLOCK("blend_transfers", {
        blended_transfer = blendTransfers(
            histogram_transfer, linear_transfer, config_.linear_percent
        );
    });
    
    // Step 5: Apply max gain limitation
    std::array<float, HISTOGRAM_BINS> limited_transfer;
    TIME_BLOCK("max_gain_limit", {
        limited_transfer = applyMaxGainLimit(blended_transfer, config_.max_gain);
    });
    
    // Step 6: Apply ACE (Adaptive Contrast Enhancement)
    std::array<float, HISTOGRAM_BINS> ace_transfer;
    TIME_BLOCK("ace_gamma", {
        ace_transfer = applyACEGamma(limited_transfer, config_.gamma);
    });
    
    // Step 7: Apply damping factor for temporal smoothing
    std::array<float, HISTOGRAM_BINS> final_transfer;
    TIME_BLOCK("damping", {
        if (first_frame_) {
            RCLCPP_DEBUG_ONCE(rclcpp::get_logger("thermal_processor"),
                         "damping: first_frame=true, bypassing temporal smoothing");
            final_transfer = ace_transfer;
            previous_transfer_ = ace_transfer;
            first_frame_ = false;
        } else {
            final_transfer = applyDamping(ace_transfer, previous_transfer_, config_.damping_factor);
            previous_transfer_ = final_transfer;
        }
    });
    
    // Step 8: Map 16-bit image to 8-bit using final transfer function
    cv::Mat image_8bit;
    TIME_BLOCK("apply_transfer", {
        image_8bit = applyTransferFunction(image_16bit, final_transfer);
    });
    
    // Step 9: Apply DDE if in plateau mode and DDE is enabled
    if (config_.process == "plateau" && config_.dde != 1.0f) {
        RCLCPP_DEBUG(rclcpp::get_logger("thermal_processor"),
                    "DDE enabled: method=%s dde=%.3f headroom=%.3f smoothing_factor=%.3f",
                    config_.process.c_str(), config_.dde, config_.detail_headroom, config_.smoothing_factor);
        TIME_BLOCK("dde_post", {
            image_8bit = applyDDEPostProcessing(image_8bit, image_16bit);
        });
    } else {
        RCLCPP_DEBUG_ONCE(rclcpp::get_logger("thermal_processor"),
                          "Skipping DDE: method=%s (needs 'plateau'), dde=%.3f (skip if == 1.0)",
                          config_.process.c_str(), config_.dde);
    }
    auto t_end = std::chrono::steady_clock::now();
    double ms = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count() / 1000.0;
    RCLCPP_DEBUG_ONCE(rclcpp::get_logger("thermal_processor"),
                 "Using CPU path (%dx%d) in %.2f ms",
                 image_16bit.cols, image_16bit.rows, ms);
    return image_8bit;
}

cv::Mat ThermalProcessor::applyTailRejection(const cv::Mat& image_16bit, float tail_percent) {
    if (tail_percent == 0.0f) {
        RCLCPP_DEBUG(rclcpp::get_logger("thermal_processor"),
                     "applyTailRejection: tail_percent=0 -> skip clipping");
        return image_16bit.clone();
    }
    
    const uint16_t* data = image_16bit.ptr<uint16_t>();
    const int total = image_16bit.rows * image_16bit.cols;
    
    // Compute asymmetric low/high cut percentages using outlier_cut_balance:
    // outlier_cut_balance in [0,2]:
    //   0 -> ignore low end  (low_cut=0,       high_cut=2*tail)
    //   1 -> balanced        (low_cut=tail,   high_cut=tail)
    //   2 -> ignore high end (low_cut=2*tail, high_cut=0)
    float ocb = static_cast<float>(config_.outlier_cut_balance);
    float low_cut  = tail_percent * std::clamp(ocb, 0.0f, 2.0f);
    float high_cut = tail_percent * (2.0f - std::clamp(ocb, 0.0f, 2.0f));
    // Convert to percentile inputs for fast finder
    float low_percent  = std::clamp(low_cut,  0.0f, 100.0f);
    float high_percent = std::clamp(100.0f - high_cut, 0.0f, 100.0f);
    
    // Use fast percentile calculation
    uint16_t low_threshold, high_threshold;
    find_percentiles_fast(data, total, low_percent, high_percent,
                         &low_threshold, &high_threshold);
    RCLCPP_DEBUG_ONCE(rclcpp::get_logger("thermal_processor"),
                      "applyTailRejection: tail=%.2f ocb=%.2f -> low_cut=%.2f%% high_cut=%.2f%% | low_thr=%u high_thr=%u",
                      tail_percent, ocb, low_cut, high_cut,
                      static_cast<unsigned>(low_threshold), static_cast<unsigned>(high_threshold));
    
    // Apply clipping in-place for better performance
    cv::Mat clipped = image_16bit.clone();
    clip_tails_fast(clipped.ptr<uint16_t>(), total,
                   low_threshold, high_threshold, num_threads_);
    
    return clipped;
}

std::array<uint32_t, ThermalProcessor::HISTOGRAM_BINS> ThermalProcessor::computeHistogram(const cv::Mat& image) {
    // Prefer GPU histogram if CUDA resources are initialized
#ifdef WITH_CUDA
    if (use_cuda_ && image.data && image.type() == CV_16UC1) {
        bool ok = cuda_agc_histogram(image.ptr<uint16_t>(), image.cols, image.rows, histogram_buffer_.get());
        if (ok) {
            std::array<uint32_t, HISTOGRAM_BINS> histogram;
            std::memcpy(histogram.data(), histogram_buffer_.get(), HISTOGRAM_BINS * sizeof(uint32_t));
            return histogram;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("thermal_processor"),
                       "cuda_agc_histogram failed, falling back to CPU histogram");
        }
    }
#endif
    // CPU fallback: fast C implementation with OpenMP
    compute_histogram_fast(image.ptr<uint16_t>(),
                           image.rows * image.cols,
                           histogram_buffer_.get(),
                           num_threads_);
    std::array<uint32_t, HISTOGRAM_BINS> histogram;
    std::memcpy(histogram.data(), histogram_buffer_.get(), HISTOGRAM_BINS * sizeof(uint32_t));
    return histogram;
}

std::array<uint64_t, ThermalProcessor::HISTOGRAM_BINS> ThermalProcessor::computeCumulativeHistogram(
    const std::array<uint32_t, HISTOGRAM_BINS>& histogram) {
    
    // Use fast C implementation
    compute_cumulative_fast(histogram.data(), cumulative_buffer_.get());
    
    std::array<uint64_t, HISTOGRAM_BINS> cumulative;
    std::memcpy(cumulative.data(), cumulative_buffer_.get(),
                HISTOGRAM_BINS * sizeof(uint64_t));
    
    return cumulative;
}

std::array<float, ThermalProcessor::HISTOGRAM_BINS> ThermalProcessor::plateauEqualization(const cv::Mat& image) {
    // Compute histogram
    auto histogram = computeHistogram(image);
    
    // Apply plateau clipping
    double total_pixels = static_cast<double>(image.rows * image.cols);
    double max_bin_population = (config_.plateau / 100.0) * total_pixels;
    RCLCPP_DEBUG(rclcpp::get_logger("thermal_processor"),
                 "plateauEqualization: plateau=%.2f%% total_pixels=%.0f max_bin=%.0f",
                 config_.plateau, total_pixels, max_bin_population);
    
    std::array<double, HISTOGRAM_BINS> clipped_histogram;
    double excess_pixels = 0.0;
    
    // Clip bins that exceed plateau
#ifdef WITH_OPENMP
    #pragma omp parallel for reduction(+:excess_pixels)
#endif
    for (size_t i = 0; i < HISTOGRAM_BINS; ++i) {
        clipped_histogram[i] = static_cast<double>(histogram[i]);
        if (clipped_histogram[i] > max_bin_population) {
            excess_pixels += clipped_histogram[i] - max_bin_population;
            clipped_histogram[i] = max_bin_population;
        }
    }
    
    // Redistribute excess pixels uniformly
    double redistribution = excess_pixels / static_cast<double>(HISTOGRAM_BINS);
    for (size_t i = 0; i < HISTOGRAM_BINS; ++i) {
        clipped_histogram[i] += redistribution;
    }
    
    // Convert back to uint32_t for cumulative calculation
    std::array<uint32_t, HISTOGRAM_BINS> clipped_hist_uint32;
    for (size_t i = 0; i < HISTOGRAM_BINS; ++i) {
        clipped_hist_uint32[i] = static_cast<uint32_t>(clipped_histogram[i]);
    }
    
    // Compute cumulative histogram
    auto cumulative = computeCumulativeHistogram(clipped_hist_uint32);
    
    // Normalize to create transfer function (maps to 0-255)
    std::array<float, HISTOGRAM_BINS> transfer_function{};
    float total_cumulative = static_cast<float>(cumulative[HISTOGRAM_BINS - 1]);
    
    if (total_cumulative > 0) {
#ifdef WITH_OPENMP
        #pragma omp parallel for
#endif
        for (size_t i = 0; i < HISTOGRAM_BINS; ++i) {
            transfer_function[i] = (static_cast<float>(cumulative[i]) / total_cumulative) * 255.0f;
        }
    }
    
    return transfer_function;
}

std::pair<cv::Mat, cv::Mat> ThermalProcessor::separateFrequencyComponents(const cv::Mat& image) {
    // Convert to float for filtering
    cv::Mat image_float;
    image.convertTo(image_float, CV_32F);
    
    // Use fast box filter approximation (much faster than Gaussian)
    cv::Mat lp_image(image.size(), CV_32F);
    int radius = static_cast<int>(config_.smoothing_factor / 500.0f * 3);
    radius = std::min(radius, 15); // Limit radius
    
    fast_gaussian_blur(image_float.ptr<float>(),
                      lp_image.ptr<float>(),
                      image.cols, image.rows, radius);
    
    // High-pass = Original - Low-pass
    cv::Mat hp_image = image_float - lp_image;
    
    return std::make_pair(hp_image, lp_image);
}

std::array<double, ThermalProcessor::HISTOGRAM_BINS> ThermalProcessor::computeWeightedHistogram(
    [[maybe_unused]] const std::array<uint32_t, HISTOGRAM_BINS>& histogram,
    const cv::Mat& hp_image, const cv::Mat& lp_image) {
    
    std::array<double, HISTOGRAM_BINS> weighted_hist{};
    
    // Find maximum absolute high-pass value for normalization
    double max_hp = 0.0;
    
    // Use OpenCV's minMaxLoc for faster computation
    double min_val, max_val;
    cv::minMaxLoc(cv::abs(hp_image), &min_val, &max_val);
    max_hp = max_val;
    
    if (max_hp == 0.0) max_hp = 1.0; // Avoid division by zero
    
    // Weight histogram by high-frequency content
#ifdef WITH_OPENMP
    const int num_threads = omp_get_max_threads();
    std::vector<std::array<double, HISTOGRAM_BINS>> thread_weighted_hists(num_threads);
    for (auto& hist : thread_weighted_hists) {
        hist.fill(0.0);
    }
    
    #pragma omp parallel
    {
        int thread_id = omp_get_thread_num();
        auto& local_hist = thread_weighted_hists[thread_id];
        
        #pragma omp for nowait
        for (int i = 0; i < lp_image.rows; ++i) {
            const float* lp_row = lp_image.ptr<float>(i);
            const float* hp_row = hp_image.ptr<float>(i);
            
            for (int j = 0; j < lp_image.cols; ++j) {
                float pixel_lp = lp_row[j];
                float pixel_hp = hp_row[j];
                
                int bin_index = static_cast<int>(clip(pixel_lp, 0.0f, 65535.0f));
                
                // Weight by normalized absolute high-pass value (range: [1.0, 2.0])
                double weight = 1.0 + std::abs(pixel_hp) / max_hp;
                local_hist[bin_index] += weight;
            }
        }
    }
    
    // Merge thread-local histograms
    for (const auto& thread_hist : thread_weighted_hists) {
        for (size_t i = 0; i < HISTOGRAM_BINS; ++i) {
            weighted_hist[i] += thread_hist[i];
        }
    }
#else
    for (int i = 0; i < lp_image.rows; ++i) {
        const float* lp_row = lp_image.ptr<float>(i);
        const float* hp_row = hp_image.ptr<float>(i);
        
        for (int j = 0; j < lp_image.cols; ++j) {
            float pixel_lp = lp_row[j];
            float pixel_hp = hp_row[j];
            
            int bin_index = static_cast<int>(clip(pixel_lp, 0.0f, 65535.0f));
            
            // Weight by normalized absolute high-pass value (range: [1.0, 2.0])
            double weight = 1.0 + std::abs(pixel_hp) / max_hp;
            weighted_hist[bin_index] += weight;
        }
    }
#endif
    
    return weighted_hist;
}

std::array<float, ThermalProcessor::HISTOGRAM_BINS> ThermalProcessor::informationBasedEqualization(const cv::Mat& image) {
    RCLCPP_DEBUG(rclcpp::get_logger("thermal_processor"),
                 "informationBasedEqualization: smoothing_factor=%.1f detail_headroom=%.2f",
                 config_.smoothing_factor, config_.detail_headroom);
    // Step 1: Separate frequency components
    auto [hp_image, lp_image] = separateFrequencyComponents(image);
    
    // Step 2: Apply plateau equalization to low-pass image
    cv::Mat lp_image_uint16;
    lp_image.convertTo(lp_image_uint16, CV_16U);
    [[maybe_unused]] auto plateau_transfer = plateauEqualization(lp_image_uint16);
    
    // Step 3: Weight histogram by high-pass content
    auto lp_histogram = computeHistogram(lp_image_uint16);
    auto weighted_histogram = computeWeightedHistogram(lp_histogram, hp_image, lp_image);
    
    // Step 4: Create transfer function from weighted histogram
    // Convert weighted histogram to uint32_t array
    std::array<uint32_t, HISTOGRAM_BINS> weighted_hist_uint32;
    for (size_t i = 0; i < HISTOGRAM_BINS; ++i) {
        weighted_hist_uint32[i] = static_cast<uint32_t>(weighted_histogram[i]);
    }
    
    auto cumulative = computeCumulativeHistogram(weighted_hist_uint32);
    std::array<float, HISTOGRAM_BINS> base_transfer{};
    float total_cumulative = static_cast<float>(cumulative[HISTOGRAM_BINS - 1]);
    
    if (total_cumulative > 0) {
#ifdef WITH_OPENMP
        #pragma omp parallel for
#endif
        for (size_t i = 0; i < HISTOGRAM_BINS; ++i) {
            base_transfer[i] = (static_cast<float>(cumulative[i]) / total_cumulative) * 255.0f;
        }
    }
    
    // Step 5: Reserve headroom for detail enhancement
    float compressed_range = 255.0f - 2.0f * config_.detail_headroom;
    std::array<float, HISTOGRAM_BINS> compressed_transfer{};
    
#ifdef WITH_OPENMP
    #pragma omp parallel for
#endif
    for (size_t i = 0; i < HISTOGRAM_BINS; ++i) {
        compressed_transfer[i] = (base_transfer[i] / 255.0f) * compressed_range + config_.detail_headroom;
    }
    
    return compressed_transfer;
}

std::array<float, ThermalProcessor::HISTOGRAM_BINS> ThermalProcessor::createLinearTransfer(const cv::Mat& image) {
    // Use fast min-max
    uint16_t min_val_uint, max_val_uint;
    find_min_max_fast(image.ptr<uint16_t>(),
                     image.rows * image.cols,
                     &min_val_uint, &max_val_uint);
    RCLCPP_DEBUG(rclcpp::get_logger("thermal_processor"),
                 "createLinearTransfer: min=%u max=%u",
                 static_cast<unsigned>(min_val_uint), static_cast<unsigned>(max_val_uint));
    
    // Avoid division by zero
    if (max_val_uint == min_val_uint) {
        max_val_uint = min_val_uint + 1;
    }
    
    // Create linear mapping
    std::array<float, HISTOGRAM_BINS> transfer{};
    float scale = 255.0f / (max_val_uint - min_val_uint);
    
    // Vectorized loop
    for (size_t i = 0; i < HISTOGRAM_BINS; ++i) {
        if (i <= min_val_uint) {
            transfer[i] = 0.0f;
        } else if (i >= max_val_uint) {
            transfer[i] = 255.0f;
        } else {
            transfer[i] = static_cast<float>(i - min_val_uint) * scale;
        }
    }
    
    return transfer;
}

std::array<float, ThermalProcessor::HISTOGRAM_BINS> ThermalProcessor::blendTransfers(
    const std::array<float, HISTOGRAM_BINS>& hist_transfer,
    const std::array<float, HISTOGRAM_BINS>& linear_transfer,
    double linear_percent) {
    
    float alpha = linear_percent / 100.0f;
    float beta = 1.0f - alpha;
    RCLCPP_DEBUG(rclcpp::get_logger("thermal_processor"),
                 "blendTransfers: linear_percent=%.1f alpha=%.3f beta=%.3f",
                 linear_percent, alpha, beta);
    
    std::array<float, HISTOGRAM_BINS> blended{};
    
#ifdef WITH_OPENMP
    #pragma omp parallel for
#endif
    for (size_t i = 0; i < HISTOGRAM_BINS; ++i) {
        blended[i] = alpha * linear_transfer[i] + beta * hist_transfer[i];
    }
    
    return blended;
}

std::array<float, ThermalProcessor::HISTOGRAM_BINS> ThermalProcessor::applyMaxGainLimit(
    const std::array<float, HISTOGRAM_BINS>& transfer, float max_gain) {
    
    std::array<float, HISTOGRAM_BINS> limited_transfer = transfer;
    int clamp_count = 0;
    
    // Sequential operation due to data dependency
    for (size_t i = 1; i < HISTOGRAM_BINS; ++i) {
        // Calculate local gain (slope)
        float local_gain = limited_transfer[i] - limited_transfer[i-1];
        
        // Limit if exceeds max_gain
        if (local_gain > max_gain) {
            limited_transfer[i] = limited_transfer[i-1] + max_gain;
            ++clamp_count;
        } else if (local_gain < -max_gain) {
            limited_transfer[i] = limited_transfer[i-1] - max_gain;
            ++clamp_count;
        }
    }
    
    // Ensure within valid range
#ifdef WITH_OPENMP
    #pragma omp parallel for
#endif
    for (size_t i = 0; i < HISTOGRAM_BINS; ++i) {
        limited_transfer[i] = clip(limited_transfer[i], 0.0f, 255.0f);
    }
    
    RCLCPP_DEBUG(rclcpp::get_logger("thermal_processor"),
                 "applyMaxGainLimit: max_gain=%.2f clamped_bins=%d",
                 max_gain, clamp_count);
    return limited_transfer;
}

std::array<float, ThermalProcessor::HISTOGRAM_BINS> ThermalProcessor::applyACEGamma(
    const std::array<float, HISTOGRAM_BINS>& transfer, float gamma) {
    
    std::array<float, HISTOGRAM_BINS> ace_transfer{};
    // Guard against invalid/zero gamma to avoid division by zero
    float clamped_gamma = std::max(gamma, 1e-3f);
    float inv_gamma = 1.0f / clamped_gamma;
    RCLCPP_DEBUG(rclcpp::get_logger("thermal_processor"),
                 "applyACEGamma: gamma=%.3f (clamped=%.3f) inv_gamma=%.3f",
                 gamma, clamped_gamma, inv_gamma);
    
    // Precompute gamma lookup table for 256 values
    std::array<float, 256> gamma_lut;
    for (int i = 0; i < 256; ++i) {
        float normalized = i / 255.0f;
        gamma_lut[i] = std::pow(normalized, inv_gamma) * 255.0f;
    }
    
#ifdef WITH_OPENMP
    #pragma omp parallel for
#endif
    for (size_t i = 0; i < HISTOGRAM_BINS; ++i) {
        // Use LUT for gamma correction
        int lut_index = static_cast<int>(clip(transfer[i], 0.0f, 255.0f));
        ace_transfer[i] = gamma_lut[lut_index];
    }
    
    return ace_transfer;
}

std::array<float, ThermalProcessor::HISTOGRAM_BINS> ThermalProcessor::applyDamping(
    const std::array<float, HISTOGRAM_BINS>& current_transfer,
    const std::array<float, HISTOGRAM_BINS>& previous_transfer,
    float damping_factor) {
    
    // Convert damping factor to IIR coefficient
    float clamped_df = std::clamp(damping_factor, 0.0f, 100.0f);
    float alpha = 1.0f - (clamped_df / 100.0f);
    // Numerically ensure alpha in [0,1]
    alpha = std::clamp(alpha, 0.0f, 1.0f);
    float beta = 1.0f - alpha;
    RCLCPP_DEBUG(rclcpp::get_logger("thermal_processor"),
                 "applyDamping: damping_factor=%.1f (clamped=%.1f) alpha=%.3f beta=%.3f",
                 damping_factor, clamped_df, alpha, beta);
    
    // IIR filter: output = alpha * current + beta * previous
    std::array<float, HISTOGRAM_BINS> damped_transfer{};
    
#ifdef WITH_OPENMP
    #pragma omp parallel for
#endif
    for (size_t i = 0; i < HISTOGRAM_BINS; ++i) {
        damped_transfer[i] = alpha * current_transfer[i] + beta * previous_transfer[i];
    }
    
    return damped_transfer;
}

cv::Mat ThermalProcessor::applyTransferFunction(const cv::Mat& image_16bit,
                                                const std::array<float, HISTOGRAM_BINS>& transfer) {
    cv::Mat image_8bit(image_16bit.size(), CV_8U);
    
    // Create lookup table for 8-bit conversion (use pre-allocated buffer)
    for (size_t i = 0; i < HISTOGRAM_BINS; ++i) {
        float pixel_8bit_float = clip(transfer[i], 0.0f, 255.0f);
        lut_buffer_[i] = static_cast<uint8_t>(std::round(pixel_8bit_float));
    }
    
    // Use fast C implementation for LUT application
    apply_transfer_fast(image_16bit.ptr<uint16_t>(),
                       image_8bit.ptr<uint8_t>(),
                       lut_buffer_.get(),
                       image_16bit.rows * image_16bit.cols,
                       num_threads_);
    
    return image_8bit;
}

cv::Mat ThermalProcessor::applyDDEPostProcessing(const cv::Mat& image_8bit, const cv::Mat& image_16bit) {
    RCLCPP_DEBUG(rclcpp::get_logger("thermal_processor"),
                "applyDDEPostProcessing: dde=%.2f detail_headroom=%.2f",
                config_.dde, config_.detail_headroom);
    // Sanity check expected types
    CV_Assert(image_8bit.type() == CV_8UC1);
    CV_Assert(image_16bit.type() == CV_16UC1);
    RCLCPP_DEBUG(rclcpp::get_logger("thermal_processor"),
                "applyDDEPostProcessing: types ok (image_8bit=%d, image_16bit=%d)",
                image_8bit.type(), image_16bit.type());
    // Extract high-frequency details from original
    auto [hp_16bit, lp_16bit] = separateFrequencyComponents(image_16bit);
    
    // Normalize high-pass to match 8-bit range with headroom
    double hp_min, hp_max;
    cv::minMaxLoc(hp_16bit, &hp_min, &hp_max);
    double hp_range = hp_max - hp_min;
    RCLCPP_DEBUG(rclcpp::get_logger("thermal_processor"),
                "applyDDEPostProcessing: hp_min=%.3f hp_max=%.3f hp_range=%.3f",
                hp_min, hp_max, hp_range);
    
    cv::Mat hp_normalized;
    if (hp_range > 1e-6) {
        // Scale to fit within detail headroom
        double scale = 2.0 * config_.detail_headroom / hp_range;
        hp_normalized = (hp_16bit - hp_min) * scale - config_.detail_headroom;
        RCLCPP_DEBUG(rclcpp::get_logger("thermal_processor"),
                    "applyDDEPostProcessing: scale=%.6f", scale);
    } else {
        hp_normalized = cv::Mat::zeros(hp_16bit.size(), CV_32F);
        RCLCPP_DEBUG(rclcpp::get_logger("thermal_processor"),
                    "applyDDEPostProcessing: hp_range too small (%.9f) -> using zeros for details", hp_range);
    }
    
    // Apply DDE gain
    cv::Mat enhanced_details = hp_normalized * config_.dde;
    // Inspect enhanced detail dynamic range
    double ed_min = 0.0, ed_max = 0.0;
    cv::minMaxLoc(enhanced_details, &ed_min, &ed_max);
    RCLCPP_DEBUG(rclcpp::get_logger("thermal_processor"),
                "applyDDEPostProcessing: enhanced_details min=%.3f max=%.3f (dde=%.3f)",
                ed_min, ed_max, config_.dde);
    
    // Add to 8-bit image
    cv::Mat enhanced_image;
    image_8bit.convertTo(enhanced_image, CV_32F);
    enhanced_image += enhanced_details;
    // Inspect pre-convert range
    double ei_min = 0.0, ei_max = 0.0;
    cv::minMaxLoc(enhanced_image, &ei_min, &ei_max);
    RCLCPP_DEBUG(rclcpp::get_logger("thermal_processor"),
                "applyDDEPostProcessing: enhanced_image (float) min=%.3f max=%.3f", ei_min, ei_max);
    
    // Clip to valid range and convert back to uint8
    cv::Mat result;
    enhanced_image.convertTo(result, CV_8U, 1.0, 0.0);
    // Optionally inspect final range (should be [0,255])
    double r_min = 0.0, r_max = 0.0;
    cv::minMaxLoc(result, &r_min, &r_max);
    RCLCPP_DEBUG(rclcpp::get_logger("thermal_processor"),
                "applyDDEPostProcessing: result (u8) min=%.0f max=%.0f", r_min, r_max);
    
    return result;
}

cv::Mat ThermalProcessor::applyColorPalette(const cv::Mat& image_8bit, uint8_t palette_id) {
    // Validate input
    if (image_8bit.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("thermal_processor"), "applyColorPalette: empty input image, returning empty");
        return image_8bit;
    }
    if (image_8bit.type() != CV_8UC1) {
        RCLCPP_WARN(rclcpp::get_logger("thermal_processor"), "applyColorPalette: expected CV_8UC1, got type=%d; converting to grayscale", image_8bit.type());
    }
    cv::Mat gray;
    if (image_8bit.type() == CV_8UC1) {
        gray = image_8bit;
    } else {
        cv::cvtColor(image_8bit, gray, cv::COLOR_BGR2GRAY);
    }

    if (palette_id == 0) {  // WhiteHot (grayscale)
        return gray;
    } else if (palette_id == 1) {  // BlackHot (inverted grayscale)
        cv::Mat inverted;
        cv::subtract(255, gray, inverted);
        return inverted;
    } else if (palette_id == 2) { // Rainbow -> use OpenCV RAINBOW colormap for speed/quality
        cv::Mat colored_image;
        cv::applyColorMap(gray, colored_image, cv::COLORMAP_RAINBOW);
        return colored_image;
    } else {
        // Load 3-channel color LUT and apply per channel
        cv::Mat color_lut = createColorLUT(palette_id); // 256x1 CV_8UC3
        if (color_lut.rows != 256 || color_lut.cols != 1 || color_lut.type() != CV_8UC3) {
            RCLCPP_ERROR(rclcpp::get_logger("thermal_processor"), "applyColorPalette: invalid LUT shape/type (rows=%d, cols=%d, type=%d), falling back to WHITEHOT", color_lut.rows, color_lut.cols, color_lut.type());
            return gray;
        }

        std::vector<cv::Mat> lut_channels;
        cv::split(color_lut, lut_channels); // B, G, R each CV_8UC1 (256x1)

        cv::Mat b, g, r;
        cv::LUT(gray, lut_channels[0], b);
        cv::LUT(gray, lut_channels[1], g);
        cv::LUT(gray, lut_channels[2], r);

        cv::Mat colored_image;
        cv::merge(std::vector<cv::Mat>{b, g, r}, colored_image); // CV_8UC3 (BGR)
        return colored_image;
    }
}

cv::Mat ThermalProcessor::createColorLUT(uint8_t palette_id) {
    cv::Mat lut(256, 1, CV_8UC3);
    
    // Simple implementation of color palettes
    // In production, these would be predefined lookup tables
    switch (palette_id) {
        case 2:  {
            // Rainbow using OpenCV colormap to generate LUT
            cv::Mat ramp(256, 1, CV_8UC1);
            for (int i = 0; i < 256; ++i) ramp.at<uchar>(i, 0) = static_cast<uchar>(i);
            cv::Mat ramp_bgr;
            cv::applyColorMap(ramp, ramp_bgr, cv::COLORMAP_RAINBOW);
            for (int i = 0; i < 256; ++i) lut.at<cv::Vec3b>(i) = ramp_bgr.at<cv::Vec3b>(i, 0);
            break;
        }
        case 3: { // Rainbow High Contrast -> use TURBO as a higher-contrast proxy
            cv::Mat ramp(256, 1, CV_8UC1);
            for (int i = 0; i < 256; ++i) ramp.at<uchar>(i, 0) = static_cast<uchar>(i);
            cv::Mat ramp_bgr;
            cv::applyColorMap(ramp, ramp_bgr, cv::COLORMAP_TURBO);
            for (int i = 0; i < 256; ++i) lut.at<cv::Vec3b>(i) = ramp_bgr.at<cv::Vec3b>(i, 0);
            break;
        }
        case 4: { // Ironbow
            for (int i = 0; i < 256; ++i) {
                cv::Vec3b& pixel = lut.at<cv::Vec3b>(i);
                if (i < 64) {
                    pixel = cv::Vec3b(0, 0, i * 4);  // Black to Blue
                } else if (i < 128) {
                    pixel = cv::Vec3b(0, (i - 64) * 4, 255);  // Blue to Cyan
                } else if (i < 192) {
                    pixel = cv::Vec3b((i - 128) * 4, 255, 255 - (i - 128) * 4);  // Cyan to Yellow
                } else {
                    pixel = cv::Vec3b(255, 255 - (i - 192) * 4, 0);  // Yellow to Red
                }
            }
            break;
        }
        case 5: { // Lava -> use INFERNO as proxy
            cv::Mat ramp(256, 1, CV_8UC1);
            for (int i = 0; i < 256; ++i) ramp.at<uchar>(i, 0) = static_cast<uchar>(i);
            cv::Mat ramp_bgr;
            cv::applyColorMap(ramp, ramp_bgr, cv::COLORMAP_INFERNO);
            for (int i = 0; i < 256; ++i) lut.at<cv::Vec3b>(i) = ramp_bgr.at<cv::Vec3b>(i, 0);
            break;
        }
        case 6: { // Arctic -> use OCEAN as proxy
            cv::Mat ramp(256, 1, CV_8UC1);
            for (int i = 0; i < 256; ++i) ramp.at<uchar>(i, 0) = static_cast<uchar>(i);
            cv::Mat ramp_bgr;
            cv::applyColorMap(ramp, ramp_bgr, cv::COLORMAP_OCEAN);
            for (int i = 0; i < 256; ++i) lut.at<cv::Vec3b>(i) = ramp_bgr.at<cv::Vec3b>(i, 0);
            break;
        }
        case 7: { // Globow -> use JET as proxy
            cv::Mat ramp(256, 1, CV_8UC1);
            for (int i = 0; i < 256; ++i) ramp.at<uchar>(i, 0) = static_cast<uchar>(i);
            cv::Mat ramp_bgr;
            cv::applyColorMap(ramp, ramp_bgr, cv::COLORMAP_JET);
            for (int i = 0; i < 256; ++i) lut.at<cv::Vec3b>(i) = ramp_bgr.at<cv::Vec3b>(i, 0);
            break;
        }
        case 8: { // Graded Fire -> use HOT as proxy
            cv::Mat ramp(256, 1, CV_8UC1);
            for (int i = 0; i < 256; ++i) ramp.at<uchar>(i, 0) = static_cast<uchar>(i);
            cv::Mat ramp_bgr;
            cv::applyColorMap(ramp, ramp_bgr, cv::COLORMAP_HOT);
            for (int i = 0; i < 256; ++i) lut.at<cv::Vec3b>(i) = ramp_bgr.at<cv::Vec3b>(i, 0);
            break;
        }
        case 9: { // Hottest -> emphasize top range, use AUTUMN as proxy (warm scale)
            cv::Mat ramp(256, 1, CV_8UC1);
            for (int i = 0; i < 256; ++i) ramp.at<uchar>(i, 0) = static_cast<uchar>(i);
            cv::Mat ramp_bgr;
            cv::applyColorMap(ramp, ramp_bgr, cv::COLORMAP_AUTUMN);
            for (int i = 0; i < 256; ++i) lut.at<cv::Vec3b>(i) = ramp_bgr.at<cv::Vec3b>(i, 0);
            break;
        }

            
        default:
            // Default to grayscale
            for (int i = 0; i < 256; ++i) {
                lut.at<cv::Vec3b>(i) = cv::Vec3b(i, i, i);
            }
            break;
    }
    
    return lut;
}

// Removed legacy applyThermalColormap (string-based). Use palette ID path instead.

cv::Mat ThermalProcessor::rectifyImage(const cv::Mat& image) {
    if (!maps_initialized_) {
        return image;
    }
    
    cv::Mat rectified;
    cv::remap(image, rectified, map1_, map2_, cv::INTER_LINEAR);
    return rectified;
}

void ThermalProcessor::initializeCudaResources(size_t width, size_t height) {
#ifdef WITH_CUDA
    // Initialize GPU buffers/stream for mapping stage
    if (width == 0 || height == 0) return;
    if (gpu_image_size_ == static_cast<size_t>(width * height)) return;
    if (cuda_agc_init(static_cast<int>(width), static_cast<int>(height))) {
        gpu_image_size_ = static_cast<size_t>(width * height);
        use_cuda_ = true;
        RCLCPP_INFO(rclcpp::get_logger("thermal_processor"), "CUDA AGC mapper initialized for %zux%zu", width, height);
    } else {
        use_cuda_ = false;
        RCLCPP_WARN(rclcpp::get_logger("thermal_processor"), "CUDA AGC init failed - using CPU path");
    }
#else
    (void)width;
    (void)height;
#endif
}

void ThermalProcessor::cleanupCudaResources() {
#ifdef WITH_CUDA
    cuda_agc_free();
    d_input_image_ = nullptr;
    d_output_image_ = nullptr;
    d_transfer_function_ = nullptr;
    d_previous_transfer_ = nullptr;
    d_histogram_ = nullptr;
    cuda_stream_ = nullptr;
    gpu_image_size_ = 0;
    use_cuda_ = false;
#endif
}

cv::Mat ThermalProcessor::applyFlirAGCCuda(const cv::Mat& image_16bit) {
#ifdef WITH_CUDA
    CV_Assert(image_16bit.type() == CV_16UC1);

    // Lazy-init GPU for the observed image size
    if (gpu_image_size_ != static_cast<size_t>(image_16bit.rows * image_16bit.cols)) {
        initializeCudaResources(static_cast<size_t>(image_16bit.cols), static_cast<size_t>(image_16bit.rows));
        if (!use_cuda_) {
            return applyFlirAGC(image_16bit);
        }
    }

    // Step 1: Tail rejection
    cv::Mat image_clipped;
    image_clipped = applyTailRejection(image_16bit, config_.tail_rejection);

    // Step 2: Build stats view (ROI or full)
    cv::Mat stats_view = image_clipped;
    if (config_.roi_enable) {
        int x = std::max(0, config_.roi_x);
        int y = std::max(0, config_.roi_y);
        int w = config_.roi_width > 0 ? config_.roi_width : image_clipped.cols;
        int h = config_.roi_height > 0 ? config_.roi_height : image_clipped.rows;
        if (x + w > image_clipped.cols) w = image_clipped.cols - x;
        if (y + h > image_clipped.rows) h = image_clipped.rows - y;
        if (w > 0 && h > 0) {
            stats_view = image_clipped(cv::Rect(x, y, w, h));
            RCLCPP_DEBUG_ONCE(rclcpp::get_logger("thermal_processor"),
                              "Software AGC ROI active (CUDA path): x=%d y=%d w=%d h=%d", x, y, w, h);
        }
    }
    // Step 2b: Histogram/information-based transfer
    std::array<float, HISTOGRAM_BINS> histogram_transfer;
    if (config_.process == "information_based") {
        histogram_transfer = informationBasedEqualization(stats_view);
    } else {
        histogram_transfer = plateauEqualization(stats_view);
    }

    // Step 3: Linear transfer (based on ROI view)
    std::array<float, HISTOGRAM_BINS> linear_transfer = createLinearTransfer(stats_view);

    // Step 4: Blend
    std::array<float, HISTOGRAM_BINS> blended_transfer = blendTransfers(
        histogram_transfer, linear_transfer, config_.linear_percent);

    // Step 5: Max gain limit
    std::array<float, HISTOGRAM_BINS> limited_transfer = applyMaxGainLimit(blended_transfer, config_.max_gain);

    // Step 6: ACE gamma
    std::array<float, HISTOGRAM_BINS> ace_transfer = applyACEGamma(limited_transfer, config_.gamma);

    // Step 7: Damping
    std::array<float, HISTOGRAM_BINS> final_transfer;
    if (first_frame_) {
        final_transfer = ace_transfer;
        previous_transfer_ = ace_transfer;
        first_frame_ = false;
    } else {
        final_transfer = applyDamping(ace_transfer, previous_transfer_, config_.damping_factor);
        previous_transfer_ = final_transfer;
    }

    // Build 8-bit LUT from final transfer into pre-allocated buffer
    for (size_t i = 0; i < HISTOGRAM_BINS; ++i) {
        float v = clip(final_transfer[i], 0.0f, 255.0f);
        lut_buffer_[i] = static_cast<uint8_t>(std::round(v));
    }

    // Allocate output and run GPU mapping
    cv::Mat image_8bit(image_16bit.size(), CV_8U);
    bool ok = cuda_agc_apply(
        image_16bit.ptr<uint16_t>(),
        image_8bit.ptr<uint8_t>(),
        lut_buffer_.get(),
        image_16bit.cols,
        image_16bit.rows);
    if (!ok) {
        RCLCPP_WARN(rclcpp::get_logger("thermal_processor"), "CUDA AGC apply failed - falling back to CPU path");
        return applyFlirAGC(image_16bit);
    }
    // Apply optional DDE on top of CUDA-mapped 8-bit (CPU-side), same gating as CPU path
    if (config_.process == "plateau" && config_.dde != 1.0f) {
        RCLCPP_DEBUG(rclcpp::get_logger("thermal_processor"),
                    "DDE (CUDA path): method=%s dde=%.3f headroom=%.3f smoothing_factor=%.3f",
                    config_.process.c_str(), config_.dde, config_.detail_headroom, config_.smoothing_factor);
        cv::Mat dde_out = applyDDEPostProcessing(image_8bit, image_16bit);
        return dde_out;
    }
    return image_8bit;
#else
    (void)image_16bit;
    return cv::Mat();
#endif
}

// ThermalProcessorNode implementation

ThermalProcessorNode::ThermalProcessorNode(const rclcpp::NodeOptions& options)
    : Node("thermal_processor", options) {
    
    declareParameters();
    loadParameters();
    
    // Initialize processor with loaded config
    processor_ = std::make_unique<ThermalProcessor>(config_);
    
    // Setup subscribers
    thermal_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/" + camera_name_ + "/image", 10,
        std::bind(&ThermalProcessorNode::thermalCallback, this, std::placeholders::_1));
    
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/" + camera_name_ + "/camera_info", 10,
        std::bind(&ThermalProcessorNode::cameraInfoCallback, this, std::placeholders::_1));
    
    // Setup publisher for processed images
    processed_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/" + camera_name_ + "/image_processed", 10);
    
    RCLCPP_INFO(this->get_logger(), "Thermal Processor Node initialized for %s", camera_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Processing mode: %s", config_.process.c_str());
    RCLCPP_INFO(this->get_logger(), "AGC Parameters: gamma=%.2f, max_gain=%.2f, damping=%.1f",
                config_.gamma, config_.max_gain, config_.damping_factor);
                
#ifdef WITH_OPENMP
    RCLCPP_INFO(this->get_logger(), "OpenMP enabled with %d threads", omp_get_max_threads());
#else
    RCLCPP_INFO(this->get_logger(), "OpenMP not available - using single thread");
#endif
}

void ThermalProcessorNode::declareParameters() {
    // Camera name
    this->declare_parameter("camera_name", "thermal");
    
    // Processing parameters
    this->declare_parameter("operation.processing.rectify", false);
    this->declare_parameter("operation.processing.enhance", true);
    this->declare_parameter("operation.processing.method", "information_based");
    
    // AGC parameters
    this->declare_parameter("operation.agc.tail_rejection", 2.0);
    this->declare_parameter("operation.agc.outlier_cut_balance", 1.0);
    this->declare_parameter("operation.agc.max_gain", 2.0);
    this->declare_parameter("operation.agc.damping_factor", 5.0);
    this->declare_parameter("operation.agc.gamma", 0.8);
    this->declare_parameter("operation.agc.plateau", 7.0);
    this->declare_parameter("operation.agc.linear_percent", 30.0);
    this->declare_parameter("operation.agc.detail_headroom", 16.0);
    this->declare_parameter("operation.agc.dde", 1.25);
    this->declare_parameter("operation.agc.smoothing_factor", 1250.0);
    this->declare_parameter("operation.agc.color_palette", 0);
}

void ThermalProcessorNode::loadParameters() {
    // Load node-specific parameters
    this->get_parameter("camera_name", camera_name_);
    
    // Load processing parameters
    this->get_parameter("operation.processing.rectify", config_.rectify);
    this->get_parameter("operation.processing.enhance", config_.enhance);
    this->get_parameter("operation.processing.method", config_.process);
    
    // Load double parameters directly
    this->get_parameter("operation.agc.tail_rejection", config_.tail_rejection);
    this->get_parameter("operation.agc.outlier_cut_balance", config_.outlier_cut_balance);
    this->get_parameter("operation.agc.max_gain", config_.max_gain);
    this->get_parameter("operation.agc.damping_factor", config_.damping_factor);
    this->get_parameter("operation.agc.gamma", config_.gamma);
    this->get_parameter("operation.agc.plateau", config_.plateau);
    this->get_parameter("operation.agc.linear_percent", config_.linear_percent);
    this->get_parameter("operation.agc.detail_headroom", config_.detail_headroom);
    this->get_parameter("operation.agc.color_palette", config_.color_palette);
    this->get_parameter("operation.agc.dde", config_.dde);
    this->get_parameter("operation.agc.smoothing_factor", config_.smoothing_factor);
    
    RCLCPP_INFO(this->get_logger(), "Loaded parameters:");
    RCLCPP_INFO(this->get_logger(), "  - Method: %s", config_.process.c_str());
    // Colormap param deprecated in this node; color palettes handled by camera node
    RCLCPP_INFO(this->get_logger(), "  - AGC: gamma=%.2f, max_gain=%.2f, damping=%.1f%%",
                config_.gamma, config_.max_gain, config_.damping_factor);
    RCLCPP_INFO(this->get_logger(), "  - Tail rejection: %.1f%%", config_.tail_rejection);
    RCLCPP_INFO(this->get_logger(), "  - Linear blend: %.1f%%", config_.linear_percent);
    RCLCPP_INFO(this->get_logger(), "  - DDE: factor=%.2f headroom=%.1f smoothing=%.1f",
                config_.dde, config_.detail_headroom, config_.smoothing_factor);
}

void ThermalProcessorNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    if (!camera_info_received_) {
        processor_->updateCalibration(msg);
        camera_info_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera calibration loaded from camera_info topic");
    }
}

void ThermalProcessorNode::thermalCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    try {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Convert ROS image to OpenCV
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "16UC1");
        cv::Mat thermal_image = cv_ptr->image;
        
        // Apply FLIR AGC processing
        cv::Mat processed_8bit = processor_->applyFlirAGC(thermal_image);
        
        // Apply rectification if enabled
        if (config_.rectify && camera_info_received_) {
            processed_8bit = processor_->rectifyImage(processed_8bit);
        }
        
        // Apply color palette using unified integer ID (0-9)
        RCLCPP_DEBUG(this->get_logger(), "Software palette ID: %d", static_cast<int>(config_.color_palette));
        cv::Mat colored_image = processor_->applyColorPalette(processed_8bit, static_cast<uint8_t>(config_.color_palette));
        
        // Convert back to ROS message and publish
        sensor_msgs::msg::Image::SharedPtr output_msg = 
            cv_bridge::CvImage(msg->header, "bgr8", colored_image).toImageMsg();
        
        processed_pub_->publish(*output_msg);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        RCLCPP_DEBUG(this->get_logger(), "Processing took %ld ms (%.1f FPS)",
                    duration.count(), 1000.0 / duration.count());
        
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Processing exception: %s", e.what());
    }
}

}  // namespace flir_ros_sync

RCLCPP_COMPONENTS_REGISTER_NODE(flir_ros_sync::ThermalProcessorNode)
