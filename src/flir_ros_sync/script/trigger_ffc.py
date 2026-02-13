#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from flirpy.camera.boson import Boson

from Boson_SDK import CamAPI
from Boson_SDK import FLR_GAO_NUC_TYPE_E

def resolve_serial_ports(serial_list, node):
    """
    Resolve each serial port to its full path using realpath and ensure it exists.
    """
    resolved_serial_ports = []
    for serial_port in serial_list:
        try:
            # Resolve the full path
            serial_port_root = os.path.realpath(f"/dev/{serial_port}")
            
            if not os.path.exists(serial_port_root):
                raise FileNotFoundError(f"Serial port {serial_port_root} cannot be resolved!")
            
            node.get_logger().info(f"Resolved serial port: {serial_port} to {serial_port_root}")
            resolved_serial_ports.append(serial_port_root)
        except Exception as e:
            node.get_logger().error(f"Error resolving serial port {serial_port}: {e}")
    
    return resolved_serial_ports

class FlirFfcTrigger(Node):
    def __init__(self):
        super().__init__('flir_ffc_trigger')

        self.get_logger().info("STARTING FFC TRIGGER NODE")

        self.declare_parameter("serial_list", ["flir_boson_serial_34582"])
        serial_list = self.get_parameter("serial_list").get_parameter_value().string_array_value
        self.get_logger().info(f"Received serial list: {serial_list}")

        # Resolve the full paths for each serial port
        self.resolved_serial_ports = resolve_serial_ports(serial_list, self)

        if not self.resolved_serial_ports:
            self.get_logger().error("No valid serial ports found, exiting...")
            return RuntimeError("No valid serial ports found")

        # Initialize cameras based on the resolved serial ports
        self.cameras_flirpy = {}
        self.cameras_boson_sdk = {}
        for serial_port in self.resolved_serial_ports:
            try:
                camera = Boson(port=serial_port)
                self.cameras_flirpy[serial_port] = camera

                # Initialize the camera using the Boson SDK
                camera = CamAPI.pyClient(manualport=serial_port)
                self.cameras_boson_sdk[serial_port] = camera

                self.get_logger().info(f"Connected to camera on port: {serial_port}")
            except Exception as e:
                self.get_logger().error(f"Failed to connect to camera on port {serial_port}: {e}")

        # self.trigger_ffc()  # Not needed because it is already triggered in flir_ros_sync.cpp after setting modes

        # Trigger FFC periodically every 3 minutes
        self.create_timer(180, self.trigger_ffc)

    def trigger_ffc(self):
        self.get_logger().info("Triggering FFC for all cameras")
        for port, camera in self.cameras_flirpy.items():
            try:
                camera.do_ffc()
            except Exception as e:
                self.get_logger().error(f"Failed FFC on camera {port}: {e}")

            try:
                # Setting NUC to type 1
                result = self.cameras_boson_sdk[port].gaoSetNucType(FLR_GAO_NUC_TYPE_E.FLR_GAO_NUC_TYPE_TWO_POINT_FIELD)
                self.get_logger().info(f"Result on {port}: {result} and NUC Type set to 1")
            except Exception as e:
                self.get_logger().error(f"Failed to set NUC on camera {port}: {e}")

            try:
                # Check if NUC table switch is desired
                if camera.get_nuc_desired() == 1:
                    self.get_logger().info(f"NUC Table Switch Desired for camera on port: {port}. Switching NUC table.")
                    camera.do_nuc_table_switch()  # Perform the NUC table switch if needed
                    result, nuc_type = self.cameras_boson_sdk[port].gaoGetNucType()
                    self.get_logger().info(f"Result on {port}: {result} and Current NUC Type : {nuc_type}")
                else:
                    self.get_logger().info(f"NUC Table Switch NOT Desired for camera on port: {port}.")
                    result, nuc_type = self.cameras_boson_sdk[port].gaoGetNucType()
                    self.get_logger().info(f"Result on {port}: {result} and Current NUC Type : {nuc_type}")
            except Exception as e:
                self.get_logger().error(f"Failed NUC on camera {port}: {e}")

    def destroy_node(self):
        # Close the cameras before exiting
        for port, camera in self.cameras_flirpy.items():
            camera.close()
        for port, camera in self.cameras_boson_sdk.items():
            camera.Close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FlirFfcTrigger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
