#ifndef OBJECT_DETECTION_H
#define OBJECT_DETECTION_H

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace object_detection {

/**
 * Disables specific transports to optimize resource usage
 * @param node ROS2 node
 * @param base_topic Base topic name
 */
inline void disable_transports(const std::shared_ptr<rclcpp::Node>& node, 
                              const std::string& base_topic) {
    // Disable unused transports to save resources
    node->declare_parameter<bool>(base_topic + ".enable_raw", true);
    node->declare_parameter<bool>(base_topic + ".enable_compressed", false);
    node->declare_parameter<bool>(base_topic + ".enable_theora", false);
}

/**
 * Helper to publish image only if there are subscribers
 * @param publisher Image publisher
 * @param image Image message
 * @param camera_info Camera info message
 */
inline void publish_if_subscribed(const image_transport::CameraPublisher& publisher,
                                 const sensor_msgs::msg::Image::SharedPtr& image,
                                 const sensor_msgs::msg::CameraInfo::SharedPtr& camera_info) {
    if (publisher.getNumSubscribers() > 0) {
        publisher.publish(image, camera_info);
    }
}

/**
 * Helper to publish image only if there are subscribers (no camera info)
 * @param publisher Image publisher
 * @param image Image message
 */
inline void publish_if_subscribed(const image_transport::Publisher& publisher,
                                 const sensor_msgs::msg::Image::SharedPtr& image) {
    if (publisher.getNumSubscribers() > 0) {
        publisher.publish(*image);
    }
}

} // namespace object_detection

#endif // OBJECT_DETECTION_H
