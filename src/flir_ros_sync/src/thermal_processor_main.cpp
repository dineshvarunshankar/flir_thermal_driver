#include <rclcpp/rclcpp.hpp>
#include "flir_ros_sync/thermal_processor.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<flir_ros_sync::ThermalProcessorNode>();
    
    // Single-threaded executor for maximum performance
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
