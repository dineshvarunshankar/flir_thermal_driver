#include "../include/flir_ros_sync/flir_ros_sync.h"
#include "flir_ros_sync/object_detection.h"

#include "../script/Boson_SDK/ClientFiles_C/EnumTypes.h"

// Standard includes
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <poll.h>
#include <cstdlib>
#include <chrono>
#include <cstring>

// V4L2 includes
#include <linux/videodev2.h>

// ROS includes
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// Project includes

namespace flir_ros_sync {

// buffer and telemetry params
size_t IMAGE_SIZE;
size_t TELEMETRY_SIZE;
size_t RAW_BUFFER_SIZE;
size_t PAGE_SIZE;
size_t ALIGNED_SIZE;
size_t TELEMETRY_OFFSET;

FlirRos::FlirRos(const rclcpp::NodeOptions& options)
    : Node("flir_ros_sync", options),
        config_{},
        device_{},
        transform_broadcaster_{this},
        stream_active_{false},
        frame_count_{0}
    {
    LOG_INFO("Constructed FLIR ROS2 Node");

    // Schedule initialize() to be called after construction is complete
    // Quick workaround to avoid calling shared_from_this() in constructor too early
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(0),
        std::bind(&FlirRos::initialize, this)
    );
}

FlirRos::~FlirRos() {
    LOG_INFO("Shutting down FLIR ROS2 Node");

    stream_active_.store(false);
    if (stream_thread_.joinable()) {
        stream_thread_.join();
    }

    // Cancel temperature monitoring timer to avoid callbacks during teardown
    if (temp_monitoring_timer_) {
        temp_monitoring_timer_->cancel();
        temp_monitoring_timer_.reset();
    }

    // Unmap V4L2 buffers to prevent memory leaks
    for (size_t i = 0; i < device_.buffers.size(); ++i) {
        if (munmap(device_.buffers[i], device_.buffer_lengths[i]) != 0) {
            LOG_ERROR("Failed to unmap buffer %zu", i);
        }
    }
    
    // Temperature monitoring timer automatically cleaned up by ROS2

    if (device_.fd >= 0) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(device_.fd, VIDIOC_STREAMOFF, &type) < 0) {
            LOG_ERROR("Failed to stop streaming");
        }
        // Close device file descriptor to avoid leaks
        if (close(device_.fd) < 0) {
            LOG_ERROR("Failed to close device fd: errno=%d (%s)", errno, strerror(errno));
        }
        device_.fd = -1;
    }
    
    // Clean up C SDK connection
    closeSDKConnection();

    // Ensure SDK shutdown is synchronized with any in-flight SDK calls
    {
        std::lock_guard<std::mutex> lock(sdk_mutex_);
        Close();
    }
}

void FlirRos::initialize() {
    LOG_INFO("Initializing FLIR ROS2 Node");

    // Cancel timer
    timer_->cancel();

    loadParameters();
    initializeDevice();
    setupROS();
    
    // Initialize in-node thermal processor for raw mode (raw-only pipeline)
    if (true) {
        auto tp_cfg = buildThermalProcessorConfig();
        processor_ = std::make_unique<ThermalProcessor>(tp_cfg);
        // Try to seed calibration if available from camera info file
        auto cam_info = publisher_.cinfo->getCameraInfo();
        if (cam_info.k.size() == 9 && !cam_info.d.empty()) {
            processor_->updateCalibration(std::make_shared<sensor_msgs::msg::CameraInfo>(cam_info));
        }
        LOG_INFO("In-node ThermalProcessor initialized for raw mode AGC");
    }
    
    // Initialize C SDK for temperature monitoring
    initializeSDKConnection();
    
    // Hardware AGC configuration disabled (raw=false path removed)
    
    // CUDA processing is now handled by separate cuda_thermal_processor node

    // Start streaming
    stream_active_ = true;
    // Initialize non-blocking gating timestamp to now
    last_publish_tp_ = std::chrono::steady_clock::now();
    // Initialize internal publish rate logger
    rate_log_start_tp_ = last_publish_tp_;
    rate_log_publish_count_ = 0;
    stream_thread_ = std::thread(&FlirRos::streamingLoop, this);
    
    // Start temperature monitoring timer (ROS2 best practice)
    // Increased interval to 10 seconds to minimize serial load during streaming
    // Create temperature monitoring timer only if enabled
    if (config_.temperature_enabled) {
        temp_monitoring_timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&FlirRos::temperatureMonitoringCallback, this)
        );
    } else {
        LOG_INFO("Temperature monitoring disabled via operation.temperature.enabled=false");
    }
}

void FlirRos::loadParameters() {
    // Declare parameters with default values
    this->declare_parameter<std::string>("device_name", "/dev/video0");
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<std::string>("camera_name", config_.camera_name);
    this->declare_parameter<std::string>("intrinsic_url", config_.intrinsic_url);
    this->declare_parameter<double>("timestamp_offset", config_.timestamp_offset);
    
    // Load operation parameters from YAML with defaults (raw-only)
    // Raw-only pipeline (force raw=true)
    this->declare_parameter("operation.raw", true);
    this->declare_parameter("operation.frame_rate", 30);
    // Software AGC parameters used by ThermalProcessor
    this->declare_parameter("operation.agc.gamma", 0.8);
    this->declare_parameter("operation.agc.max_gain", 2.0);
    this->declare_parameter("operation.agc.linear_percent", 30.0);
    this->declare_parameter("operation.agc.plateau", 7.0);
    this->declare_parameter("operation.agc.tail_rejection", 2.0);
    this->declare_parameter("operation.agc.outlier_cut_balance", 1.0);
    this->declare_parameter("operation.agc.dde", 1.25);
    this->declare_parameter("operation.agc.detail_headroom", 16.0);
    this->declare_parameter("operation.agc.smoothing_factor", 1250.0);
    this->declare_parameter("operation.agc.damping_factor", 5.0);
    this->declare_parameter("operation.agc.color_palette", 0);
    // Unified AGC ROI selector for software pipeline: full | center50 | center30
    this->declare_parameter("operation.agc.roi", std::string("full"));
    // Short alias preferred for CLI/launch
    this->declare_parameter("agc_roi", std::string("full"));
    // Processing toggles (software)
    this->declare_parameter("operation.processing.rectify", false);
    this->declare_parameter("operation.processing.enhance", true);
    this->declare_parameter("operation.processing.method", "information_based");
    // Internal publish rate logger interval (seconds); <=0 disables logging
    this->declare_parameter("operation.publish.rate_log_interval", 5.0);
    // Temperature monitoring
    this->declare_parameter("operation.temperature.enabled", true);
    this->declare_parameter("operation.temperature.warning_threshold", 60.0);
    this->declare_parameter("operation.temperature.critical_threshold", 70.0);
    
    // Declare short parameter names for backward compatibility with launch files
    this->declare_parameter("agc_smoothing_factor", 1250.0);
    // Deprecated ROI fields retained for back-compat but not recommended
    this->declare_parameter("agc_roi_enable", false);
    this->declare_parameter("agc_roi_x", 0);
    this->declare_parameter("agc_roi_y", 0);
    this->declare_parameter("agc_roi_width", 0);
    this->declare_parameter("agc_roi_height", 0);
    
    // Get device-specific parameters
    this->get_parameter("device_name", device_.device_path);
    this->get_parameter("serial_port", device_.serial_port);
    this->get_parameter("camera_name", config_.camera_name);
    this->get_parameter("intrinsic_url", config_.intrinsic_url);
    this->get_parameter("timestamp_offset", config_.timestamp_offset);
    
    // Get operation parameters (try YAML config first, fallback to launch args)
    // Force raw=true regardless of YAML/CLI to simplify single-camera pipeline
    config_.raw = true;
    if (!this->get_parameter("operation.frame_rate", config_.frame_rate)) {
        this->get_parameter("frame_rate", config_.frame_rate);
    }
    // Unified ROI mode (prefer short alias)
    if (!this->get_parameter("agc_roi", config_.agc_roi_mode)) {
        this->get_parameter("operation.agc.roi", config_.agc_roi_mode);
    }
    // Temperature polling enable/disable
    this->get_parameter("operation.temperature.enabled", config_.temperature_enabled);
    // Read internal rate logger interval
    this->get_parameter("operation.publish.rate_log_interval", rate_log_interval_sec_);
    
    // Load processing controls
    this->get_parameter("operation.processing.rectify", config_.processing_rectify);
    this->get_parameter("operation.processing.enhance", config_.processing_enhance);
    this->get_parameter("operation.processing.method", config_.processing_method);

    // Palette is controlled by operation.agc.color_palette (ID 0-9). Legacy string 'colormap' is no longer used.
    
    // Set fixed values that were previously hardcoded
    config_.width = 640;
    config_.height = 512;
    // Telemetry disabled in raw-only pipeline; total_height equals height
    config_.total_height = config_.height;
    // Log raw-only mode for clarity
    RCLCPP_INFO(this->get_logger(), "Raw-only pipeline enabled (operation.raw=true; telemetry disabled)");
    config_.send_every_n = 1;    // Deprecated: decimation now uses time-based gating
    
    // External sync removed; no sync or FFC settings are loaded
    
    // Get temperature thresholds
    float temp_warning, temp_critical;
    this->get_parameter("operation.temperature.warning_threshold", temp_warning);
    this->get_parameter("operation.temperature.critical_threshold", temp_critical);
    
    // Store temperature thresholds for monitoring
    this->set_parameter(rclcpp::Parameter("operation.temperature.warning_threshold", temp_warning));
    this->set_parameter(rclcpp::Parameter("operation.temperature.critical_threshold", temp_critical));

    this->get_parameter("device_name", device_.device_path);
    this->get_parameter("serial_port", device_.serial_port);
    this->get_parameter("camera_name", config_.camera_name);
    this->get_parameter("intrinsic_url", config_.intrinsic_url);
    this->get_parameter("width", config_.width);
    this->get_parameter("height", config_.height);
    // Do NOT read short-name 'raw' here; 'operation.raw' is the authoritative param.
    // send_every_n is no longer a user parameter; publish rate is time-gated by operation.frame_rate
    this->get_parameter("timestamp_offset", config_.timestamp_offset);
    this->get_parameter("frame_rate", config_.frame_rate);
    // ffc_interval not used in raw-only pipeline

    // Initialize time-based publish gating period from desired frame rate
    int desired_fps = config_.frame_rate > 0 ? config_.frame_rate : 60;
    publish_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / static_cast<double>(desired_fps))
    );
    
    // Get AGC parameters - load from YAML for both raw=true and raw=false
    LOG_INFO("Loading AGC parameters from YAML config");
    // Try to get from YAML config first (operation.agc.*), fallback to launch args if not found
    if (!this->get_parameter("operation.agc.gamma", config_.agc_gamma)) {
        this->get_parameter("agc_gamma", config_.agc_gamma);
    }
    if (!this->get_parameter("operation.agc.max_gain", config_.agc_max_gain)) {
        this->get_parameter("agc_max_gain", config_.agc_max_gain);
    }
    // Load double parameters directly
    if (!this->get_parameter("operation.agc.linear_percent", config_.agc_linear_percent)) {
        this->get_parameter("agc_linear_percent", config_.agc_linear_percent);
    }
    
    if (!this->get_parameter("operation.agc.plateau", config_.agc_plateau)) {
        this->get_parameter("agc_plateau", config_.agc_plateau);
    }
    
    if (!this->get_parameter("operation.agc.tail_rejection", config_.agc_tail_rejection)) {
        this->get_parameter("agc_tail_rejection", config_.agc_tail_rejection);
    }
    
    if (!this->get_parameter("operation.agc.outlier_cut_balance", config_.agc_outlier_cut_balance)) {
        this->get_parameter("agc_outlier_cut_balance", config_.agc_outlier_cut_balance);
    }
    
    if (!this->get_parameter("operation.agc.dde", config_.agc_dde)) {
        this->get_parameter("agc_dde", config_.agc_dde);
    }
    
    if (!this->get_parameter("operation.agc.detail_headroom", config_.agc_detail_headroom)) {
        this->get_parameter("agc_detail_headroom", config_.agc_detail_headroom);
    }
    
    if (!this->get_parameter("operation.agc.smoothing_factor", config_.agc_smoothing_factor)) {
        this->get_parameter("agc_smoothing_factor", config_.agc_smoothing_factor);
    }
    
    if (!this->get_parameter("operation.agc.damping_factor", config_.agc_damping_factor)) {
        this->get_parameter("agc_damping_factor", config_.agc_damping_factor);
    }
    
    // Robustly load color palette (supports int or string params)
    bool got_palette = this->get_parameter("operation.agc.color_palette", config_.agc_color_palette);
    if (!got_palette) {
        // Try alt name
        got_palette = this->get_parameter("agc_color_palette", config_.agc_color_palette);
    }
    if (!got_palette) {
        // Try to read as generic parameter and convert if it's a string
        rclcpp::Parameter p;
        if (this->get_parameter("operation.agc.color_palette", p)) {
            if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                config_.agc_color_palette = std::stoi(p.as_string());
                got_palette = true;
            } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                config_.agc_color_palette = static_cast<int>(p.as_int());
                got_palette = true;
            }
        }
    }
    if (!got_palette) {
        // Final fallback: check alt name as string
        rclcpp::Parameter p2;
        if (this->get_parameter("agc_color_palette", p2)) {
            if (p2.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                config_.agc_color_palette = std::stoi(p2.as_string());
                got_palette = true;
            } else if (p2.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                config_.agc_color_palette = static_cast<int>(p2.as_int());
                got_palette = true;
            }
        }
    }
    this->get_parameter("deployment_profile", config_.deployment_profile);
    
    // Palette is controlled by operation.agc.color_palette (ID 0-9). Legacy string 'colormap' is no longer used.

LOG_INFO("Loaded parameters");
LOG_INFO("AGC Color Palette (id): %d", config_.agc_color_palette);
LOG_INFO("Device name: %s", device_.device_path.c_str());
LOG_INFO("Serial port: %s", device_.serial_port.c_str());
LOG_INFO("Camera name: %s", config_.camera_name.c_str());
LOG_INFO("Intrinsic URL: %s", config_.intrinsic_url.c_str());
LOG_INFO("Frame rate: %d", config_.frame_rate);
{
    LOG_INFO("Image height: %d (telemetry disabled)", config_.height);
    LOG_INFO("Timestamp offset: %f", config_.timestamp_offset);
    
    // Log AGC parameters to verify source
    LOG_INFO("=== AGC PARAMETERS LOADED ===");
    LOG_INFO("AGC Gamma: %.2f", config_.agc_gamma);
    LOG_INFO("AGC Max Gain: %.2f", config_.agc_max_gain);
    LOG_INFO("AGC Linear Percent: %.1f", config_.agc_linear_percent);
    LOG_INFO("AGC Plateau: %.1f", config_.agc_plateau);
    LOG_INFO("AGC Tail Rejection: %.1f", config_.agc_tail_rejection);
    LOG_INFO("AGC Outlier Cut Balance: %.1f", config_.agc_outlier_cut_balance);
    LOG_INFO("AGC DDE: %.2f", config_.agc_dde);
    LOG_INFO("AGC Detail Headroom: %.2f", config_.agc_detail_headroom);
    LOG_INFO("AGC Smoothing Factor: %.0f", config_.agc_smoothing_factor);
    LOG_INFO("AGC Damping Factor: %.1f", config_.agc_damping_factor);
    LOG_INFO("Processing Method: %s (rectify=%s, enhance=%s)",
             config_.processing_method.c_str(),
             config_.processing_rectify ? "true" : "false",
             config_.processing_enhance ? "true" : "false");
    // Hardware palette/ROI/deployment profile logging removed (raw=false path removed)
    LOG_INFO("==============================");
    LOG_INFO("Frame rate: %d", config_.frame_rate);
 }
 
 }
 
 void FlirRos::initializeDevice() {
    // Resolve device path - use dynamic allocation to prevent buffer overflow
    char* device_realpath = realpath(device_.device_path.c_str(), nullptr);
    if (!device_realpath) {
        throw std::runtime_error("Failed to resolve device path");
    }
    device_.device_path = std::string(device_realpath);
    free(device_realpath);  // Free the allocated memory
    LOG_INFO("Device path resolved to %s", device_.device_path.c_str());

    // Check if device is already in use (prevent multiple instances)
    device_.fd = open(device_.device_path.c_str(), O_RDWR | O_NONBLOCK);
    if (device_.fd < 0) {
        if (errno == EBUSY) {
            throw std::runtime_error("FLIR camera is already in use by another process. Please stop the existing node first.");
        } else {
            throw std::runtime_error("Failed to open device: " + std::string(strerror(errno)));
        }
    }
    
    // Remove non-blocking flag for normal operation
    int flags = fcntl(device_.fd, F_GETFL);
    fcntl(device_.fd, F_SETFL, flags & ~O_NONBLOCK);

    // Verify streaming capabilities
    struct v4l2_capability cap;
    std::memset(&cap, 0, sizeof(cap));
    if (ioctl(device_.fd, VIDIOC_QUERYCAP, &cap) < 0 ||
        !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        throw std::runtime_error("Device cannot stream video");
    }

    // Resolve serial port
    char* serial_realpath = realpath(device_.serial_port.c_str(), nullptr);
    if (!serial_realpath) {
        throw std::runtime_error("Failed to resolve serial port");
    }
    device_.serial_port = std::string(serial_realpath);
    free(serial_realpath);  // Free the allocated memory
    LOG_INFO("Serial port resolved to %s", device_.serial_port.c_str());

    // Configure Gain and FFC modes
    // FFC and Gain configuration via legacy UART helpers removed.
    // Boson handles FFC internally in auto mode; no software tracking or forcing here.

    //Initialize FLIR SDK

    // Open device
    std::string port_name = device_.serial_port;
    // Lookup port number using a helper function using the port name
    int32_t port_num = FSLP_lookup_port_id(const_cast<char*>(port_name.c_str()), port_name.length());
    if (port_num == -1) {
        // Handle error: Port not found
        throw std::runtime_error("Invalid serial port: " + port_name);
    }
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    FLR_RESULT result = Initialize(port_num, 921600);
    if (result != FLR_COMM_OK) {
        LOG_ERROR("Failed to initialize FLIR SDK. Error code: %d", result);
        throw std::runtime_error("FLIR SDK initialization failed");
    }
    LOG_INFO("FLIR SDK initialized successfully");

    // Telemetry initialization disabled in raw-only pipeline
    LOG_INFO("Telemetry initialization skipped (raw-only pipeline)");

    // Initialize size variables for raw-only (Y16) pipeline
    IMAGE_SIZE = config_.width * config_.height * 2;        // 16-bit per pixel (Y16)
    TELEMETRY_SIZE = 0;                                     // Telemetry disabled
    RAW_BUFFER_SIZE = IMAGE_SIZE;                           // No telemetry appended
    PAGE_SIZE = sysconf(_SC_PAGE_SIZE);
    ALIGNED_SIZE = ((RAW_BUFFER_SIZE + PAGE_SIZE - 1) / PAGE_SIZE) * PAGE_SIZE;
    TELEMETRY_OFFSET = IMAGE_SIZE;  // Unused when telemetry disabled

    // Set format and request buffers
    // The following function responsible for requesting enough buffer to handle the image and telemetry:
    if (!setFormat(device_.fd)) {
        throw std::runtime_error("Failed to set video format");
    }
    // If successful, the following function will request a buffer of the correct size:
    if (!requestBuffers(device_.fd)) {
        throw std::runtime_error("Failed to request buffers");
    }
    if (!startStreaming(device_.fd)) {
        throw std::runtime_error("Failed to start streaming");
    }
}

void FlirRos::setupROS() {
    // Initialize image transport and camera info manager
    publisher_.it = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    publisher_.cinfo = std::make_shared<camera_info_manager::CameraInfoManager>(
        this, config_.camera_name, config_.intrinsic_url);

    // Set up topic names and frame IDs
    publisher_.base_frame_id = config_.camera_name + "/camera_link";
    publisher_.img_opt_frame_id = config_.camera_name + "/optical_frame";
    
    // Raw-only: publish raw 16-bit and processed 8-bit topics
    publisher_.camera_topic_name = config_.camera_name + "/image";
    // Advertise raw image topic
    publisher_.image_pub = publisher_.it->advertiseCamera(publisher_.camera_topic_name, 10);
    // Also advertise processed image topic for in-node AGC
    publisher_.processed_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        config_.camera_name + std::string("/image_processed"), 10);
    // We don't create rect_image_pub anymore as it's not needed


    // Init timestamp publisher


    // FFC status topic removed (auto FFC handled internally)
    
    // Init temperature publishers only if enabled
    if (config_.temperature_enabled) {
        publisher_.core_temp_pub_ = this->create_publisher<std_msgs::msg::Float32>(config_.camera_name+"/core_temperature", 10);
        publisher_.fpa_temp_pub_ = this->create_publisher<std_msgs::msg::Float32>(config_.camera_name+"/fpa_temperature", 10);
    } else {
        RCLCPP_INFO(this->get_logger(), "Temperature polling disabled: not advertising temperature topics");
    }
    
    // Note: CUDA and rect processing removed - thermal_processor node handles all processing
}

// Raw-only pipeline; telemetry and manual FFC are not used

void FlirRos::streamingLoop() {
    LOG_INFO("Starting streaming thread");
    struct v4l2_buffer bufferinfo;

    // Auto FFC mode enabled - no manual timing needed
    // ffc_frame_threshold = ffc_frame_count;

    while (stream_active_.load()) {
        // Wait for frame readiness using poll() to avoid blocking forever and handle timeouts
        struct pollfd pfd;
        pfd.fd = device_.fd;
        pfd.events = POLLIN | POLLPRI;
        int poll_ret = poll(&pfd, 1, 1000); // 1000 ms
        if (poll_ret == 0) {
            // Timeout: no frame within 1s. Keep running but record it and optionally restart on repeated timeouts.
            consecutive_timeouts_++;
            RCLCPP_INFO(this->get_logger(), "Timeout: 1.000000 s");
            if (consecutive_timeouts_ >= 5) {
                restartStream("poll timeout");
                consecutive_timeouts_ = 0;
            }
            continue;
        } else if (poll_ret < 0) {
            int err = errno;
            LOG_ERROR("poll() failed: errno=%d (%s)", err, strerror(err));
            if (!restartStream("poll failure")) {
                break;
            }
            continue;
        }

        // Dequeue the next filled buffer
        std::memset(&bufferinfo, 0, sizeof(bufferinfo));
        bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufferinfo.memory = V4L2_MEMORY_MMAP;
        if (ioctl(device_.fd, VIDIOC_DQBUF, &bufferinfo) < 0) {
            int err = errno;
            consecutive_dq_errors_++;
            LOG_ERROR("Failed to dequeue buffer: errno=%d (%s) [consecutive=%d]",
                      err, strerror(err), consecutive_dq_errors_);
            // Attempt recovery on transient errors, otherwise restart stream
            if (err == EINTR || err == EAGAIN) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            if (!restartStream("VIDIOC_DQBUF error")) {
                break;
            }
            consecutive_dq_errors_ = 0;
            continue;
        }

        // Successful DQBUF clears timeout/error counters
        consecutive_timeouts_ = 0;
        consecutive_dq_errors_ = 0;

        // Set current buffer pointer for downstream code (compatibility)
        if (bufferinfo.index < static_cast<unsigned int>(device_.buffers.size())) {
            device_.buffer = device_.buffers[bufferinfo.index];
        }

        frame_count_++;

        // Auto FFC mode handles FFC automatically - no manual intervention needed

        // Non-blocking time-based publish gating
        // Skip early frames (first 3) and then publish only if the target period has elapsed
        if (frame_count_ > 3) {
            auto now_tp = std::chrono::steady_clock::now();
            if (now_tp - last_publish_tp_ >= publish_period_) {
                // Advance by whole periods to maintain cadence even when frames are uneven
                do {
                    last_publish_tp_ += publish_period_;
                } while (now_tp - last_publish_tp_ >= publish_period_);

                // Use node clock for frame timestamp (telemetry disabled)
                rclcpp::Time frame_time = this->now();
                publishFrame(bufferinfo.bytesused, frame_time);
                publishTransforms(frame_time);
                // Temperature monitoring moved to separate thread to prevent streaming delays
            }
        }

        // Auto FFC mode - status is handled automatically by camera firmware
        
        // Re-queue the processed buffer back to driver
        // Use a fresh struct for QBUF per V4L2 best practices (MMAP requires only type/memory/index)
        struct v4l2_buffer qbuf;
        std::memset(&qbuf, 0, sizeof(qbuf));
        qbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        qbuf.memory = V4L2_MEMORY_MMAP;
        qbuf.index = bufferinfo.index;

        int attempts = 0;
        const int max_attempts = 5;
        while (true) {
            if (ioctl(device_.fd, VIDIOC_QBUF, &qbuf) == 0) {
                break;  // success
            }

            int err = errno;
            attempts++;
            LOG_ERROR("Failed to re-queue buffer index %d (attempt %d/%d): errno=%d (%s)",
                      qbuf.index, attempts, max_attempts, err, strerror(err));

            // Retry on common transient errors
            if ((err == EINTR || err == EAGAIN) && attempts < max_attempts) {
                // Small backoff (0.5-2 ms) to yield; avoid blocking the entire thread
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // If streaming has been stopped asynchronously, exit the loop gracefully
            if (err == EIO) {
                LOG_WARN("Driver reported EIO during QBUF; assuming stream shutdown");
                stream_active_.store(false);
                break;
            }

            // Non-recoverable error
            break;
        }

        // If QBUF loop ended with an error, attempt a restart; otherwise clear error counter
        if (attempts >= max_attempts) {
            consecutive_q_errors_++;
            if (!restartStream("VIDIOC_QBUF repeated failure")) {
                break;
            }
            consecutive_q_errors_ = 0;
        } else {
            consecutive_q_errors_ = 0;
        }
    }
}

bool FlirRos::setFormat(int fd) {
    struct v4l2_format format;
    std::memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = config_.width;
    format.fmt.pix.height = config_.total_height;  // Use total height including telemetry
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;  // Raw-only Y16 pipeline
    format.fmt.pix.sizeimage = ALIGNED_SIZE;

    if (ioctl(fd, VIDIOC_S_FMT, &format) < 0) {
        LOG_ERROR("Failed to set format with size %zu", ALIGNED_SIZE);
        return false;
    }

    // Verify format
    struct v4l2_format verify_format;
    std::memset(&verify_format, 0, sizeof(verify_format));
    verify_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (ioctl(fd, VIDIOC_G_FMT, &verify_format) < 0) {
        LOG_ERROR("Failed to verify format");
        return false;
    }

    if (verify_format.fmt.pix.height != static_cast<__u32>(config_.total_height)) {
        LOG_ERROR("Failed to set correct height. Requested: %d, Got: %d",
                 config_.total_height, verify_format.fmt.pix.height);
        return false;
    }

    LOG_INFO("Format set successfully: %dx%d", 
             verify_format.fmt.pix.width, verify_format.fmt.pix.height);

    return true;
}

bool FlirRos::requestBuffers(int fd) {
    // Request multiple MMAP buffers
    struct v4l2_requestbuffers req;
    std::memset(&req, 0, sizeof(req));
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    req.count = 6; // increased ring size to reduce back-pressure and QBUF contention

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        LOG_ERROR("Failed to request buffers");
        return false;
    }

    if (req.count < 2) {
        LOG_WARN("Driver provided only %d buffer(s); performance may be limited", req.count);
    }

    // Clear any existing vectors
    device_.buffers.clear();
    device_.buffer_lengths.clear();
    device_.buffers.reserve(req.count);
    device_.buffer_lengths.reserve(req.count);

    // Cleanup helper to unmap already-mapped buffers on failure paths
    auto cleanup_mapped = [this]() {
        for (size_t j = 0; j < device_.buffers.size(); ++j) {
            if (device_.buffers[j] != nullptr) {
                if (munmap(device_.buffers[j], device_.buffer_lengths[j]) != 0) {
                    LOG_ERROR("Failure cleanup: munmap buffer %zu failed", j);
                }
            }
        }
        device_.buffers.clear();
        device_.buffer_lengths.clear();
        device_.buffer_count = 0;
        device_.buffer = nullptr;
    };

    size_t total_len = 0;
    for (unsigned int i = 0; i < req.count; ++i) {
        struct v4l2_buffer query_buffer;
        std::memset(&query_buffer, 0, sizeof(query_buffer));
        query_buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        query_buffer.memory = V4L2_MEMORY_MMAP;
        query_buffer.index = i;

        if (ioctl(fd, VIDIOC_QUERYBUF, &query_buffer) < 0) {
            LOG_ERROR("Failed to query buffer %u", i);
            cleanup_mapped();
            return false;
        }

        // Verify buffer size is sufficient (image + optional telemetry)
        const size_t required_bytes = (size_t)RAW_BUFFER_SIZE; // equals IMAGE_SIZE + TELEMETRY_SIZE
        if ((size_t)query_buffer.length < required_bytes) {
            LOG_ERROR("Buffer %u size too small: got %d bytes, need %zu bytes", i, query_buffer.length, required_bytes);
            cleanup_mapped();
            return false;
        }

        // Map buffer
        void* buf_ptr = mmap(nullptr, query_buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, query_buffer.m.offset);
        if (buf_ptr == MAP_FAILED) {
            LOG_ERROR("Failed to mmap buffer %u", i);
            cleanup_mapped();
            return false;
        }

        std::memset(buf_ptr, 0, query_buffer.length);
        device_.buffers.push_back(buf_ptr);
        device_.buffer_lengths.push_back(static_cast<size_t>(query_buffer.length));
        total_len += static_cast<size_t>(query_buffer.length);

        // Queue buffer initially so the driver can fill it
        struct v4l2_buffer qbuf;
        std::memset(&qbuf, 0, sizeof(qbuf));
        qbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        qbuf.memory = V4L2_MEMORY_MMAP;
        qbuf.index = i;
        if (ioctl(fd, VIDIOC_QBUF, &qbuf) < 0) {
            LOG_ERROR("Failed to queue buffer %u", i);
            cleanup_mapped();
            return false;
        }
    }

    device_.buffer_count = req.count;
    // Keep first buffer pointer for compatibility with existing code paths
    device_.buffer = device_.buffers.empty() ? nullptr : device_.buffers[0];

    LOG_INFO("Allocated %d MMAP buffers, total %zu bytes (each >= RAW_BUFFER_SIZE %zu; image %zu, telemetry %zu)",
             device_.buffer_count, total_len, static_cast<size_t>(RAW_BUFFER_SIZE), static_cast<size_t>(IMAGE_SIZE), static_cast<size_t>(TELEMETRY_SIZE));
    return true;
}

bool FlirRos::startStreaming(int fd) {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        LOG_ERROR("Failed to start streaming");
        return false;
    }
    return true;
}

bool FlirRos::restartStream(const char* reason) {
    LOG_WARN("Attempting to restart V4L2 stream due to: %s", reason ? reason : "unknown");
    if (device_.fd < 0) {
        LOG_ERROR("restartStream: invalid device fd");
        return false;
    }

    // Stop the stream (ignore errors)
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(device_.fd, VIDIOC_STREAMOFF, &type) < 0) {
        int err = errno;
        LOG_WARN("STREAMOFF returned errno=%d (%s) - continuing", err, strerror(err));
        // proceed anyway
    }

    // Re-queue all mapped buffers
    for (int i = 0; i < device_.buffer_count; ++i) {
        struct v4l2_buffer qbuf;
        std::memset(&qbuf, 0, sizeof(qbuf));
        qbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        qbuf.memory = V4L2_MEMORY_MMAP;
        qbuf.index = static_cast<__u32>(i);
        if (ioctl(device_.fd, VIDIOC_QBUF, &qbuf) < 0) {
            int err = errno;
            LOG_ERROR("restartStream: QBUF failed for index %d: errno=%d (%s)", i, err, strerror(err));
            // If we cannot queue all, try to continue; driver might still accept STREAMON if enough are queued
        }
    }

    // Start streaming again
    if (!startStreaming(device_.fd)) {
        LOG_ERROR("restartStream: STREAMON failed");
        return false;
    }
    LOG_INFO("V4L2 stream successfully restarted");
    return true;
}



// Telemetry timestamp extraction removed (raw-only pipeline)

void FlirRos::publishFrame([[maybe_unused]] uint32_t bytes_used, const rclcpp::Time& time) {
    auto img = std::make_shared<sensor_msgs::msg::Image>();
    img->width = config_.width;
    img->height = config_.height;
    img->is_bigendian = false;
    img->header.stamp = time;
    img->header.frame_id = publisher_.img_opt_frame_id;

    auto cam_info = std::make_shared<sensor_msgs::msg::CameraInfo>(publisher_.cinfo->getCameraInfo());
    cam_info->header = img->header;

    if (true) {
        // Publish 16-bit raw thermal data
        img->encoding = "16UC1";
        img->step = config_.width * 2;
        img->data.assign(
            static_cast<uint8_t*>(device_.buffer),
            static_cast<uint8_t*>(device_.buffer) + IMAGE_SIZE
        );
        
        // Publish raw 16-bit image
        object_detection::publish_if_subscribed(publisher_.image_pub, img, cam_info);

        // In-node AGC: compute processed 8-bit image and publish
        if (processor_ && publisher_.processed_pub_) {
            // Wrap SDK buffer (exclude telemetry lines) as CV_16UC1
            cv::Mat image_16bit(config_.height, config_.width, CV_16UC1, device_.buffer);
            cv::Mat image_8bit = processor_->applyFlirAGC(image_16bit);

            // Decide publishing format based on AGC color_palette
            // Grayscale palettes => mono8 (0=WhiteHot, 1=BlackHot)
            const int palette = static_cast<int>(config_.agc_color_palette);
            const bool grayscale_palette = (palette == 0 || palette == 1);
            RCLCPP_DEBUG(this->get_logger(), "AGC palette=%d, grayscale=%s", palette, grayscale_palette ? "true" : "false");

            if (grayscale_palette) {
                // Apply grayscale palette operations (e.g., BlackHot inversion)
                cv::Mat gray_out;
                if (palette == 1) {
                    // BlackHot: invert grayscale
                    cv::bitwise_not(image_8bit, gray_out);
                } else {
                    gray_out = image_8bit; // WhiteHot or other grayscale variants
                }

                auto processed = std::make_shared<sensor_msgs::msg::Image>();
                processed->width = static_cast<uint32_t>(gray_out.cols);
                processed->height = static_cast<uint32_t>(gray_out.rows);
                processed->is_bigendian = false;
                processed->encoding = "mono8";
                processed->step = static_cast<uint32_t>(gray_out.cols);
                processed->header = img->header;
                size_t bytes = static_cast<size_t>(gray_out.total());
                processed->data.resize(bytes);
                std::memcpy(processed->data.data(), gray_out.data, bytes);
                publisher_.processed_pub_->publish(*processed);
            } else {
                // Apply color palette and publish as BGR8
                cv::Mat image_bgr = processor_->applyColorPalette(image_8bit, static_cast<uint8_t>(palette));
                auto processed = std::make_shared<sensor_msgs::msg::Image>();
                processed->width = static_cast<uint32_t>(image_bgr.cols);
                processed->height = static_cast<uint32_t>(image_bgr.rows);
                processed->is_bigendian = false;
                processed->encoding = "bgr8";
                processed->step = static_cast<uint32_t>(image_bgr.cols * 3);
                processed->header = img->header;
                size_t bytes = static_cast<size_t>(image_bgr.total() * image_bgr.elemSize());
                processed->data.resize(bytes);
                std::memcpy(processed->data.data(), image_bgr.data, bytes);
                publisher_.processed_pub_->publish(*processed);
            }
        }
        
    }

    // Transform publishing removed - not needed for single camera setup
}

void FlirRos::publishTransform(const rclcpp::Time& time, const geometry_msgs::msg::Vector3& trans,
                               const tf2::Quaternion& q, const std::string& from,
                               const std::string& to) {
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = time;
    transform_msg.header.frame_id = from;
    transform_msg.child_frame_id = to;
    transform_msg.transform.translation = trans;
    transform_msg.transform.rotation = tf2::toMsg(q);
    transform_broadcaster_.sendTransform(transform_msg);
}

void FlirRos::publishTransforms(const rclcpp::Time& time) {
    tf2::Quaternion q;
    q.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
    geometry_msgs::msg::Vector3 zero_translation{};
    publishTransform(time, zero_translation, q, publisher_.base_frame_id, publisher_.img_opt_frame_id);
}

ThermalProcessor::Config FlirRos::buildThermalProcessorConfig() const {
    ThermalProcessor::Config cfg;
    cfg.rectify = config_.processing_rectify;
    cfg.enhance = config_.processing_enhance;
    cfg.process = config_.processing_method;
    cfg.tail_rejection = config_.agc_tail_rejection;
    cfg.max_gain = config_.agc_max_gain;
    cfg.damping_factor = config_.agc_damping_factor;
    cfg.gamma = config_.agc_gamma;
    cfg.plateau = config_.agc_plateau;
    cfg.linear_percent = config_.agc_linear_percent;
    cfg.outlier_cut_balance = config_.agc_outlier_cut_balance;
    cfg.detail_headroom = config_.agc_detail_headroom;
    cfg.dde = config_.agc_dde;
    cfg.smoothing_factor = config_.agc_smoothing_factor;
    cfg.color_palette = config_.agc_color_palette;
    // Derive software ROI based on agc_roi_mode (full, center50, center30, center20)
    int img_w = config_.width;
    int img_h = config_.height;
    std::string mode = config_.agc_roi_mode;
    for (auto &c : mode) c = static_cast<char>(::tolower(c));
    int x=0, y=0, w=img_w, h=img_h;
    if (mode == "center50") {
        w = static_cast<int>(img_w * 0.5);
        h = static_cast<int>(img_h * 0.5);
        x = (img_w - w) / 2;
        y = (img_h - h) / 2;
        cfg.roi_enable = true;
    } else if (mode == "center30") {
        w = static_cast<int>(img_w * 0.3);
        h = static_cast<int>(img_h * 0.3);
        x = (img_w - w) / 2;
        y = (img_h - h) / 2;
        cfg.roi_enable = true;
    } else if (mode == "center20") {
        w = static_cast<int>(img_w * 0.2);
        h = static_cast<int>(img_h * 0.2);
        x = (img_w - w) / 2;
        y = (img_h - h) / 2;
        cfg.roi_enable = true;
    } else {
        cfg.roi_enable = false;
    }
    cfg.roi_x = x;
    cfg.roi_y = y;
    cfg.roi_width = w;
    cfg.roi_height = h;
    return cfg;
}

float FlirRos::getCurrentCoreTemperature() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return core_temperature_;
}

float FlirRos::getCurrentFPATemperature() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return fpa_temperature_;
}

// C SDK Temperature monitoring implementation
bool FlirRos::initializeSDKConnection() {
    // Initialize SDK connection for temperature monitoring
    // The SDK connection is handled through the existing serial port connection
    // which is already established in initializeDevice()
    LOG_INFO("C SDK temperature monitoring initialized");
    return true;
}

void FlirRos::closeSDKConnection() {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    // SDK connection cleanup is handled in destructor
    LOG_INFO("C SDK temperature monitoring closed");
}

float FlirRos::getCoreTemperature() {
    float myriad_temp = -999.0f;
    
    // Use C SDK function to get core temperature
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    FLR_RESULT result = bosonGetMyriadTemp(&myriad_temp);
    
    if (result != R_SUCCESS) {
        // Error 622 is expected during FFC operations, suppress logging for this case
        if (result != 622) {
            LOG_WARN("Failed to get core temperature. SDK Error code: %d", result);
        }
        // Note: Error 622 (FFC in progress) is normal camera behavior, no logging needed
        return -999.0f;
    } 
    
    // Temperature validation removed - direct temperature reading
    
    return myriad_temp;
}

float FlirRos::getFPATemperature() {
    int16_t fpa_temp_raw = -9999;
    
    // Use C SDK function to get FPA temperature (in degrees C x 10)
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    FLR_RESULT result = bosonlookupFPATempDegCx10(&fpa_temp_raw);
    
    if (result != R_SUCCESS) {
        // Error 622 is expected during FFC operations, suppress logging for this case
        if (result != 622) {
            LOG_WARN("Failed to get FPA temperature. SDK Error code: %d", result);
        }
        // Note: Error 622 (FFC in progress) is normal camera behavior, no logging needed
        return -999.0f;
    }  
    // Temperature validation removed - direct temperature reading
    
    // Convert from degrees C x 10 to degrees C with 1 decimal precision
    return static_cast<float>(fpa_temp_raw) / 10.0f;
}

// Temperature validation removed - checkTemperatureReadiness function obsolete

void FlirRos::temperatureMonitoringCallback() {
    // ROS2 timer-based temperature monitoring (best practice)
    // Acquire SDK temperatures without holding data_mutex_ to avoid lock coupling
    float new_core_temp = getCoreTemperature();
    float new_fpa_temp = getFPATemperature();

    // Update shared state under data_mutex_
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (new_core_temp != -999.0f) {
            core_temperature_ = new_core_temp;
        }
        if (new_fpa_temp != -999.0f) {
            fpa_temperature_ = new_fpa_temp;
        }
    }

    // Publish current values (publishTemperatures() takes the lock internally)
    publishTemperatures();
}

void FlirRos::updateTemperatures() {
    // Get SDK temperatures first (uses sdk_mutex_) without holding data_mutex_
    float new_core_temp = getCoreTemperature();
    float new_fpa_temp = getFPATemperature();

    // Update shared values under data_mutex_
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (new_core_temp != -999.0f) {
        core_temperature_ = new_core_temp;
    }
    if (new_fpa_temp != -999.0f) {
        fpa_temperature_ = new_fpa_temp;
    }
}

void FlirRos::publishTemperatures() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!config_.temperature_enabled) {
        return; // Nothing to do
    }
    // Publish core temperature
    if (publisher_.core_temp_pub_) {
        std_msgs::msg::Float32 core_temp_msg;
        core_temp_msg.data = core_temperature_;
        publisher_.core_temp_pub_->publish(core_temp_msg);
    }
    // Publish FPA temperature
    if (publisher_.fpa_temp_pub_) {
        std_msgs::msg::Float32 fpa_temp_msg;
        fpa_temp_msg.data = fpa_temperature_;
        publisher_.fpa_temp_pub_->publish(fpa_temp_msg);
    }
}

}  // namespace flir_ros_sync

RCLCPP_COMPONENTS_REGISTER_NODE(flir_ros_sync::FlirRos)
