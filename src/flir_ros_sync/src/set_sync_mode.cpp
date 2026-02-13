#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "rawBoson.h"
#include <stdlib.h> // for char *realpath(const char *restrict path, char *restrict resolved_path); to read symlink for thermal serial
#include <stdio.h>
#include <iostream>

#define CHECK_FATAL(x, err, node)     \
    if ((x))                          \
    {                                 \
        RCLCPP_FATAL(node->get_logger(), err); \
        return 1;                     \
    }

int main(int argc, char **argv)
{
    // Initialize ROS2 node
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("set_sync_mode");

    // Declare and get parameters
    int syncMode = node->declare_parameter<int>("sync_mode", 0); // default to disabled mode

    std::vector<std::string> serialList;
    node->declare_parameter<std::vector<std::string>>("serial_list", serialList);
    node->get_parameter("serial_list", serialList);

    if (serialList.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "serial_list is empty!");
        rclcpp::shutdown();
        return 1;
    }

    for (const std::string& serialPort : serialList)
    {
        char serialPortRoot[1024];
        char *serialResult = realpath((serialPort).c_str(), serialPortRoot);
        
        std::string errorMsg = "Serial port " + serialPort + " cannot be resolved!";
        CHECK_FATAL(!serialResult, errorMsg.c_str(), node);
        
        set_sync_mode(syncMode, serialPortRoot);
        int returnedSyncMode = get_sync_mode(serialPortRoot);

        switch (returnedSyncMode)
        {
        case 0:
            RCLCPP_INFO(node->get_logger(), "%s returns mode setting result: disabled.", serialPort.c_str());
            break;

        case 1:
            RCLCPP_INFO(node->get_logger(), "%s returns mode setting result: master.", serialPort.c_str());
            break;

        case 2:
            RCLCPP_INFO(node->get_logger(), "%s returns mode setting result: slave.", serialPort.c_str());
            break;

        default:
            RCLCPP_WARN(node->get_logger(), "%s returns an unknown mode setting result: %d", serialPort.c_str(), returnedSyncMode);
            break;
        }
    }

    rclcpp::shutdown();
    return 0;
}
