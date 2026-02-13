#!/usr/bin/python3

import rospy
import yaml
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import rosbag


class FlirInfoPublisher:
    def __init__(self):
        thermal_img_topic = rospy.get_param("~thermal_topic")
        caminfo_topic = rospy.get_param("~camera_info_topic")
        self.intrinsic_url = rospy.get_param("~intrinsic_url")
        self.frame_id = rospy.get_param("~thermal_frame_id")

        # configure camera_info msg
        self.camera_info_msg = CameraInfo()
        self.set_camera_info()
        self.caminfo_pub = rospy.Publisher(
            caminfo_topic, CameraInfo, queue_size=1
        )

        rospy.Subscriber(thermal_img_topic, Image, self.publish_info)

    def set_camera_info(self):
        # Load data from file
        with open(self.intrinsic_url, "r") as file_handle:
            calib_data = yaml.load(file_handle)
        # Parse
        self.camera_info_msg.width = calib_data["image_width"]
        self.camera_info_msg.height = calib_data["image_height"]
        self.camera_info_msg.K = calib_data["camera_matrix"]["data"]
        self.camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
        self.camera_info_msg.R = calib_data["rectification_matrix"]["data"]
        self.camera_info_msg.P = calib_data["projection_matrix"]["data"]
        self.camera_info_msg.distortion_model = calib_data["distortion_model"]

    def publish_info(self, data):
        # configure image msg
        stamp = data.header.stamp
        # add header
        self.camera_info_msg.header.frame_id = self.frame_id
        self.camera_info_msg.header.stamp = stamp
        # publish
        self.caminfo_pub.publish(self.camera_info_msg)


if __name__ == "__main__":
    rospy.init_node("flir_info_publisher")
    flir_info_pub = FlirInfoPublisher()
    rospy.spin()
