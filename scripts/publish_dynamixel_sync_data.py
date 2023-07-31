#!/usr/bin/env python3

#*******************************************************************************
# Copyright 2023 @ arachakonda.github.io
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
#*******************************************************************************

#*******************************************************************************
# This example is written for DYNAMIXEL XC-330-M288-T series with U2D2.
# This example specifically demonstrates the position control of DYNAMIXEL while
# using velocity profile in position control mode(single rev) by using step-
# increments
# Open terminal #1
# $ roscore
#
# Open terminal #2
# $ rosrun ros_dynamixel readWrite_pos_current.py
#
# Open terminal #3 (run one of below commands at a time)
#
#
# Author: Ananth Rachakonda
#******************************************************************************/

#*******************************************************************************
# The flow of the code is as follows:
# 1) first open port using the portHandler object to communcate with U2D2
# 2) set the baudrate of that port to communcate with DYNAMIXELs
# 3) set operation mode of the DYNAMIXELs to POSITION_CONTROL_MODE
# 4) enable torque on the DYNAMIXELs
# 5) add bulkRead Parameters; note that you can add the starting address and
#    and read multiple parameters of the DYNAMIXEL based on length specified
# 6) create a message object posCurr
# 7) home the DYNAMIXELs
# 8) start the publisher for position and current
#******************************************************************************/
import os
import rospy
from dynamixel_sdk import *
from ros_dynamixel.msg import *
from ros_dynamixel.vars import *
from ros_dynamixel.utils import *
from ros_dynamixel.comms import *
import time

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_CURR, LEN_PRESENT_DATA)
DXL_IDS = [1, 2, 3, 4, 5, 6, 7, 8]
time.sleep(0.5)

def init_id(pos,vel,curr,id):
    pos.id = id
    vel.id = id
    curr.id = id

def syncRead_pos_vel_curr(groupSyncRead, DXL_IDS, pos, vel, curr):
    dxl_comm_result = groupSyncRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    


def pub_sync_pos_vel_curr(packetHandler, groupSyncRead, DXL_IDS, pos, vel, curr):
    pub_pos = rospy.Publisher('positionSync', PositionSync, queue_size=10)
    pub_vel = rospy.Publisher('velocitySync', VelocitySync, queue_size=10)
    pub_curr = rospy.Publisher('currentSync', CurrentSync, queue_size=10)
    rospy.init_node('pub_pos_vel_curr', anonymous=True)
    ros_freq = 10
    rate = rospy.Rate(ros_freq) # 10hz
    while not rospy.is_shutdown():
        #read data from dynamixels
        syncRead_pos_vel_curr(groupSyncRead, DXL_IDS, pos, vel, curr)
        #publish data
        pub_pos.publish(pos)
        pub_vel.publish(vel)
        pub_curr.publish(curr)
        #sleep for enough time to match the frequency of publishing
        rate.sleep()

def main():
    open_port(portHandler)
    set_baudrate(portHandler, BAUDRATE)
    #add IDs for sync read of same data from multiple dynamixels
    add_SyncReadIDs(groupSyncRead, DXL_IDS)
    #sync read data from multiple dynamixels
    try:
        pos = Position()
        vel = Velocity()
        curr = Current()
        pub_sync_pos_vel_curr(packetHandler, groupSyncRead, DXL_IDS, pos, vel, curr)
    except rospy.ROSInterruptException:
        print("ROS Node Terminated")


if __name__ == '__main__':
    main()