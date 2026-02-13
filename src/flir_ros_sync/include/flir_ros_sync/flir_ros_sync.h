#ifndef FLIR_ROS_SYNC_H
#define FLIR_ROS_SYNC_H

// Standard includes
#include <memory>
#include <atomic>
#include <thread>
#include <string>
#include <ctime>
#include <functional>
#include <chrono>
 #include <vector>
#include <mutex>

// CUDA processing for AGC is available via ThermalProcessor (GPU-accelerated path)

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

// OpenCV includes
#include <opencv2/opencv.hpp>

// Project includes
#include "fd_guard.h"
// Thermal processing (AGC) inside camera node when raw=true
#include "flir_ros_sync/thermal_processor.h"

extern "C" {
    #include "EnumTypes.h"
    #include "ReturnCodes.h"
    #include "Client_API.h"
    #include "UART_Connector.h"
    #include "serialPortAdapter.h"
    #include "Serializer_Struct.h"
}

namespace flir_ros_sync {

// Color codes
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

struct CameraConfig {
    std::string camera_name = "flir";
    std::string intrinsic_url = "package://flir_ros_sync/data/camera_info/flir_intrinsics.yaml";
    int width = 640;
    int height = 512;
    int total_height = 514;
    bool raw = true;
    int gain_mode = 2;
    int ffc_mode = 1;
    int send_every_n = 1;
    double timestamp_offset = 0.0;
    int frame_rate = 10;
    int ffc_interval_mins = 3; // default 3 minutes
    bool temperature_enabled = true;    // Enable/disable periodic temperature polling
    
    // Hardware AGC parameters (only used when raw=false)
    double agc_gamma = 0.8;              // Adaptive Contrast Enhancement (ACE): recommended <1 for more feature pop (0.5-4.0 valid)
    double agc_max_gain = 2.0;           // Max Gain limits max contrast scaling; 2.0 recommended for detail preservation (0.25-8.0 valid)
    double agc_linear_percent = 30.0;         // Linear percent (0-100), 30% helps both hot/cold feature mapping and subtle anomaly preservation
    double agc_plateau = 7.0;                 // Plateau (1-100), sets the percent of pixels allowed in a histogram bin; 7 is default
    double agc_tail_rejection = 2.0;          // Tail rejection (0-50), ignores top/bottom % histogram to reject hot/cold outliers
    double agc_outlier_cut_balance = 1.0;     // Outlier cut balance (0=ignore low end, 1=balanced [default], 2=ignore high end)
    double agc_dde = 1.25;               // Digital Detail Enhancement (0.8-2.0), sharpens edges. 1.0 default, 1.25 for more detail
    double agc_detail_headroom = 16.0;   // Detail Headroom (0-255), typical values are 8-24. Promotes local edge detail when DDE enabled
    double agc_smoothing_factor = 1250.0;// Smoothing Factor (recommended default: 1250), higher values preserve finer edges when using DDE
    double agc_damping_factor = 5.0;          // Damping factor for AGC adaptation (0-100, 0=fast AGC, 100=almost frozen); 5 is fast but smooth
    int agc_color_palette = 0;                // Software/hardware palette ID (0-9)
    std::string deployment_profile = "ground";  // Deployment profile: ground, air, custom

    // Unified AGC ROI selection (applies to both modes)
    // Values: "full" (default), "center50", "center30"
    std::string agc_roi_mode = "full";
    // Derived pixel ROI (computed per-mode behind the scenes)
    bool agc_roi_enable = false;
    int agc_roi_x = 0;
    int agc_roi_y = 0;
    int agc_roi_width = 0;
    int agc_roi_height = 0;

    // Processing controls (software pipeline)
    bool processing_rectify = false;
    bool processing_enhance = true;
    std::string processing_method = "information_based"; // "information_based" or "plateau"
};

struct DeviceInfo {
    int fd = -1;
    void* buffer = nullptr;
    // Multi-buffer MMAP support
    std::vector<void*> buffers;           // Pointers to all mapped buffers
    std::vector<size_t> buffer_lengths;   // Length of each mapped buffer
    int buffer_count = 0;                 // Number of queued/mapped buffers
    std::string device_path;
    std::string serial_port;
};

struct PublisherContext {
    // Temperature publishers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr core_temp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fpa_temp_pub_;
    
    // Processed image publisher (mono8/BGR8)
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_pub_;

    std::shared_ptr<image_transport::ImageTransport> it;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo;
    image_transport::CameraPublisher image_pub;

    std::string camera_topic_name;
    std::string base_frame_id;
    std::string img_opt_frame_id;
};

class FlirRos : public rclcpp::Node{
public:
    explicit FlirRos(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~FlirRos();

    void initialize();

private:
    // Telemetry/extern sync removed; timestamps use ROS time
    
    // Temperature monitoring
    float core_temperature_{25.0f};    // Core temperature in Celsius (initialized to room temp)
    float fpa_temperature_{25.0f};     // FPA temperature in Celsius (initialized to room temp)
    uint32_t temp_frame_counter_{0};    // Frame counter for temperature polling
    bool first_frame_{true};
    
    // In-node thermal processing (AGC) resources
    std::unique_ptr<ThermalProcessor> processor_;

    // Mutex for synchronizing access to shared data (e.g., temperatures)
    std::mutex data_mutex_;
    std::mutex sdk_mutex_;

    // FFC tracking removed (auto FFC handled internally)

    // CV Mat maps
    cv::Mat map1_, map2_; // Rectification maps
    bool maps_initialized_{false};

    // Non-blocking time-based publish gating
    std::chrono::steady_clock::time_point last_publish_tp_{};
    std::chrono::steady_clock::duration publish_period_{};

    // Internal publish rate logging
    std::chrono::steady_clock::time_point rate_log_start_tp_{};
    uint64_t rate_log_publish_count_{0};
    double rate_log_interval_sec_{5.0};

    // Initialization methods
    void loadParameters();
    void initializeDevice();
    void setupROS();
    // Telemetry initialization removed

    // Streaming methods
    void streamingLoop();
    void temperatureMonitoringCallback();
    void publishFrame(uint32_t bytes_used, const rclcpp::Time& time);
    void publishTransforms(const rclcpp::Time& time);
    void publishTransform(const rclcpp::Time& time, const geometry_msgs::msg::Vector3& trans,
                          const tf2::Quaternion& q, const std::string& from,
                          const std::string& to);
    // void getFrameTime(rclcpp::Time& frame_time);

    // Utility methods
    bool setFormat(int fd);
    bool requestBuffers(int fd);
    bool startStreaming(int fd);
    // Robustness helper: attempt to restart V4L2 stream without tearing down the node
    bool restartStream(const char* reason);
    
    // C SDK Temperature monitoring methods
    bool initializeSDKConnection();
    void closeSDKConnection();
    float getCoreTemperature();
    float getFPATemperature();
    void updateTemperatures();
    
    // Hardware AGC/palette removed (raw=false path removed)

    void publishTemperatures();

    // Helper to build ThermalProcessor configuration from node params
    ThermalProcessor::Config buildThermalProcessorConfig() const;
    
    
    // Public getter functions for temperature values (for frame processing)
    float getCurrentCoreTemperature();
    float getCurrentFPATemperature();

    // Camera configuration and device info
    CameraConfig config_;
    DeviceInfo device_;
    PublisherContext publisher_;

    // ROS tools
    tf2_ros::TransformBroadcaster transform_broadcaster_;
    image_geometry::PinholeCameraModel cam_model_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Streaming control
    std::atomic<bool> stream_active_{false};
    std::thread stream_thread_;
    // Watchdog counters for robustness
    int consecutive_timeouts_{0};
    int consecutive_q_errors_{0};
    int consecutive_dq_errors_{0};
    
    // Temperature monitoring timer (ROS2 best practice)
    rclcpp::TimerBase::SharedPtr temp_monitoring_timer_;
    int frame_count_ = 0;

    // Logging macros
    #define LOG_INFO(...) RCLCPP_INFO(this->get_logger(), ANSI_COLOR_GREEN "[INFO] " ANSI_COLOR_RESET __VA_ARGS__)
    #define LOG_ERROR(...) RCLCPP_ERROR(this->get_logger(), ANSI_COLOR_RED "[ERROR] " ANSI_COLOR_RESET __VA_ARGS__)
    #define LOG_FATAL(...) RCLCPP_FATAL(this->get_logger(), ANSI_COLOR_RED "[FATAL] " ANSI_COLOR_RESET __VA_ARGS__)
    #define LOG_WARN(...) RCLCPP_WARN(this->get_logger(), ANSI_COLOR_YELLOW "[WARN] " ANSI_COLOR_RESET __VA_ARGS__)
};

}  // namespace flir_ros_sync

#endif  // FLIR_ROS_SYNC_H
