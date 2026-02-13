#!/usr/bin/env python3
import os
import rospy
from Boson_SDK import *


def set_slave_mode(serial_port_device_path, sync_mode=FLR_BOSON_EXT_SYNC_MODE_E.FLR_BOSON_EXT_SYNC_DISABLE_MODE):
    UART = pyClient.Initialize(
        manualport=os.path.realpath(serial_port_device_path))

    # set external sync mode
    result = pyClient.bosonSetExtSyncMode(sync_mode)
    print("Set external trigger mode", sync_mode, "for", serial_port_device_path, ":", result)

    pyClient.Close(UART)


if __name__ == "__main__":
    rospy.init_node("set_sync_mode")

    sync_mode_code = rospy.get_param(
        "~sync_mode", default=0)
    # dummy setting
    if sync_mode_code == 0:
        sync_mode = FLR_BOSON_EXT_SYNC_MODE_E.FLR_BOSON_EXT_SYNC_DISABLE_MODE
    elif sync_mode_code == 1:
        sync_mode = FLR_BOSON_EXT_SYNC_MODE_E.FLR_BOSON_EXT_SYNC_MASTER_MODE
    elif sync_mode_code == 2:
        sync_mode = FLR_BOSON_EXT_SYNC_MODE_E.FLR_BOSON_EXT_SYNC_SLAVE_MODE

    serial_symlink_list = rospy.get_param(
        "~serial_list", default=None)

    if serial_symlink_list is None:
        rospy.logfatal("serial_list is empty!")

    for serial_symlink in serial_symlink_list:
        set_slave_mode(serial_port_device_path=serial_symlink,
                       sync_mode=sync_mode)
