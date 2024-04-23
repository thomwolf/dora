#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

#
# *********     Read and Write Example      *********
#
#
# Available DXL model on this example : All models using Protocol 1.0
# This example is tested with a DXL MX-28, and an USB2DYNAMIXEL
# Be sure that DXL MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
#

import os

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

from dora import Node
from dora import DoraStatus

import numpy as np
from utils import LABELS


CI = os.environ.get("CI")

# Control table address
ADDR_MX_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 116
ADDR_MX_PRESENT_POSITION   = 132

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 1                 # Dynamixel ID : 1
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/tty.usbserial-FT8ISO38'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 1023           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 3073            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position


class Plotter:
    """
    Control servo from bounding box
    """

    def __init__(self):
        self.bboxs = []
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()


        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")


    def on_input(
        self,
        dora_input,
    ) -> DoraStatus:
        """

        Args:
            dora_input["id"] (str): Id of the dora_input declared in the yaml configuration
            dora_input["value"] (arrow array): message of the dora_input
        """
        print("on_input")
        if dora_input["id"] == "bbox":
            bboxs = dora_input["value"].to_numpy()
            print("got bboxs:", bboxs)
            self.bboxs = np.reshape(bboxs, (-1, 10))
        for bbox in self.bboxs:
            [
                min_x,
                min_y,
                max_x,
                max_y,
                min_xn,
                min_yn,
                max_xn,
                max_yn,
                confidence,
                label,
            ] = bbox

            label_txt = LABELS[int(label)]

            if label_txt == "cell phone":
                print("cell phone seen")
                # Write goal position
                goal_position = int((DXL_MAXIMUM_POSITION_VALUE - DXL_MINIMUM_POSITION_VALUE) * (min_xn + max_xn) / 2 + DXL_MINIMUM_POSITION_VALUE)
                print("going to position: ", goal_position)
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, goal_position)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                    break

        return DoraStatus.CONTINUE

    def stop(self):
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Close port
        self.portHandler.closePort()


plotter = Plotter()
node = Node()

for event in node:
    event_type = event["type"]
    if event_type == "INPUT":
        status = plotter.on_input(event)
        if status == DoraStatus.CONTINUE:
            pass
        elif status == DoraStatus.STOP:
            print("plotter returned stop status")
            break
    elif event_type == "STOP":
        print("received stop")
        plotter.stop()
    else:
        print("received unexpected event:", event_type)

        plotter.stop()


