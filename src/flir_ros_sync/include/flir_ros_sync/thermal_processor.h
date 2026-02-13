#ifndef THERMAL_PROCESSOR_H
#define THERMAL_PROCESSOR_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <map>
#include <array>
#include <cstdint>

#include "flir_ros_sync/gpu_agc_mapper.h"

namespace flir_ros_sync {

/**
 * @brief Thermal image processing class implementing FLIR AGC algorithm
 * Converts 16-bit thermal data (0-65535) to 8-bit display data (0-255)
 */
class ThermalProcessor {
public:
    // Constants
    static constexpr uint16_t MAX_16BIT_VALUE = 65535;
    static constexpr uint8_t MAX_8BIT_VALUE = 255;
    static constexpr size_t HISTOGRAM_BINS = 65536;

    struct Config {
        // Basic processing parameters
        bool rectify = false;
        bool enhance = true;
        std::string process = "information_based";  // "information_based" or "plateau"
        
        // Full AGC parameters from YAML
        double tail_rejection = 2.0;      // [0.0, 49.0] percentage of histogram tails to exclude
        double max_gain = 2.0;            // [0.25, 8.0] maximum transfer function slope
        double damping_factor = 5.0;      // [0.0, 100.0] temporal smoothing strength
        double gamma = 0.8;               // [0.5, 4.0] ACE gamma correction value
        double plateau = 7.0;             // [1.0, 100.0] plateau equalization limit percentage
        double linear_percent = 30.0;     // [0.0, 100.0] linear vs histogram blend ratio
        double outlier_cut_balance = 1.0; // [0.0, 2.0] outlier cut balance (0=ignore low end, 1=balanced, 2=ignore high end)
        double detail_headroom = 16.0;     // [0, 127] reserved range for details
        double dde = 1.25;                // [0.0, 6.0] digital detail enhancement factor
        double smoothing_factor = 1250.0; // [1.0, 8191.0] high/low pass filter cutoff
        int color_palette = 0;             // [0, 9] color palette ID
        // ROI for AGC stats (software path). When enabled, histograms are computed on ROI
        bool roi_enable = false;
        int roi_x = 0;
        int roi_y = 0;
        int roi_width = 0;
        int roi_height = 0;

        // Camera calibration
        cv::Mat K;  // Camera intrinsic matrix
        cv::Mat distortion;  // Distortion coefficients
    };

    explicit ThermalProcessor(const Config& config);
    ~ThermalProcessor();

    /**
     * @brief Update configuration parameters
     */
    void updateConfig(const Config& config);

    /**
     * @brief Update camera calibration from camera_info message
     */
    void updateCalibration(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info);

    /**
     * @brief Main AGC processing function - converts 16-bit to 8-bit
     * @param image_16bit 16-bit thermal image
     * @return 8-bit processed image
     */
    cv::Mat applyFlirAGC(const cv::Mat& image_16bit);
    
    /**
     * @brief GPU-accelerated AGC processing (if CUDA available)
     */
    cv::Mat applyFlirAGCCuda(const cv::Mat& image_16bit);

    /**
     * @brief Apply color palette by ID
     * @param image_8bit 8-bit grayscale image
     * @param palette_id Palette ID (0-9)
     * @return Colored image (grayscale or RGB)
     */
    cv::Mat applyColorPalette(const cv::Mat& image_8bit, uint8_t palette_id);

    /**
     * @brief Rectify thermal image using camera calibration
     * @param image Input thermal image
     * @return Rectified image
     */
    cv::Mat rectifyImage(const cv::Mat& image);

private:
    Config config_;
    
    // Transfer function storage (65536 entries for 16-bit to 8-bit mapping)
    std::array<float, HISTOGRAM_BINS> transfer_function_;
    std::array<float, HISTOGRAM_BINS> previous_transfer_; // For temporal damping
    bool first_frame_ = true;
    
    // Pre-allocated working buffers for performance
    std::unique_ptr<uint32_t[]> histogram_buffer_;
    std::unique_ptr<uint64_t[]> cumulative_buffer_;
    std::unique_ptr<uint8_t[]> lut_buffer_;
    int num_threads_ = 1;
    
    // CUDA resources
    bool use_cuda_ = false;
    uint16_t* d_input_image_ = nullptr;
    uint8_t* d_output_image_ = nullptr;
    float* d_transfer_function_ = nullptr;
    float* d_previous_transfer_ = nullptr;
    uint32_t* d_histogram_ = nullptr;
    void* cuda_stream_ = nullptr;
    size_t gpu_image_size_ = 0;
    
    // Camera calibration
    cv::Mat map1_, map2_;
    bool maps_initialized_ = false;
    
    // Core AGC processing functions
    cv::Mat applyTailRejection(const cv::Mat& image_16bit, float tail_percent);
    std::array<float, HISTOGRAM_BINS> computeTransferFunction(const cv::Mat& image_clipped);
    std::array<float, HISTOGRAM_BINS> plateauEqualization(const cv::Mat& image);
    std::array<float, HISTOGRAM_BINS> informationBasedEqualization(const cv::Mat& image);
    std::array<float, HISTOGRAM_BINS> createLinearTransfer(const cv::Mat& image);
    std::array<float, HISTOGRAM_BINS> blendTransfers(const std::array<float, HISTOGRAM_BINS>& hist_transfer,
                                                     const std::array<float, HISTOGRAM_BINS>& linear_transfer,
                                                     double linear_percent);
    std::array<float, HISTOGRAM_BINS> applyMaxGainLimit(const std::array<float, HISTOGRAM_BINS>& transfer, float max_gain);
    std::array<float, HISTOGRAM_BINS> applyACEGamma(const std::array<float, HISTOGRAM_BINS>& transfer, float gamma);
    std::array<float, HISTOGRAM_BINS> applyDamping(const std::array<float, HISTOGRAM_BINS>& current_transfer,
                                                   const std::array<float, HISTOGRAM_BINS>& previous_transfer,
                                                   float damping_factor);
    cv::Mat applyTransferFunction(const cv::Mat& image_16bit, const std::array<float, HISTOGRAM_BINS>& transfer);
    cv::Mat applyDDEPostProcessing(const cv::Mat& image_8bit, const cv::Mat& image_16bit);
    
    // Helper functions
    std::array<uint32_t, HISTOGRAM_BINS> computeHistogram(const cv::Mat& image);
    std::array<uint64_t, HISTOGRAM_BINS> computeCumulativeHistogram(const std::array<uint32_t, HISTOGRAM_BINS>& histogram);
    std::pair<cv::Mat, cv::Mat> separateFrequencyComponents(const cv::Mat& image);
    std::array<double, HISTOGRAM_BINS> computeWeightedHistogram(const std::array<uint32_t, HISTOGRAM_BINS>& histogram,
                                                               const cv::Mat& hp_image, const cv::Mat& lp_image);
    cv::Mat createColorLUT(uint8_t palette_id);
    void initializeRectificationMaps();
    
    // Clipping helper
    template<typename T>
    T clip(T value, T min_val, T max_val) {
        return std::max(min_val, std::min(value, max_val));
    }
    
    // CUDA resource management
    void initializeCudaResources(size_t width, size_t height);
    void cleanupCudaResources();
};

/**
 * @brief ROS2 node for thermal image processing
 */
class ThermalProcessorNode : public rclcpp::Node {
public:
    explicit ThermalProcessorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ThermalProcessorNode() = default;

private:
    // ROS2 parameters
    void declareParameters();
    void loadParameters();
    
    // Callbacks
    void thermalCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
    
    // Members
    std::unique_ptr<ThermalProcessor> processor_;
    ThermalProcessor::Config config_;
    std::string camera_name_;
    bool camera_info_received_ = false;
    
    // ROS2 subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr thermal_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_pub_;
};

}  // namespace flir_ros_sync

#endif  // THERMAL_PROCESSOR_H
