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

groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)
groupBulkRead = GroupBulkRead(portHandler, packetHandler)

time.sleep(2)

def init_id(pos,vel,curr,id):
    pos.id = id
    vel.id = id
    curr.id = id

def read_pos_vel_curr(groupBulkRead,packetHandler,id,pos,vel,curr):
    init_id(pos,vel,curr,id)
    bulkRead(groupBulkRead, packetHandler)
    pos.position = extractData(groupBulkRead, id, ADDR_PRESENT_POS, LEN_PRESENT_POS)
    vel.velocity = extractData(groupBulkRead, id, ADDR_PRESENT_VEL, LEN_PRESENT_VEL)
    curr.current = extractData(groupBulkRead, id, ADDR_PRESENT_CURR, LEN_PRESENT_CURR)
    

def pub_pos_vel_curr(packetHandler, groupBulkRead, id, pos, vel, curr):
    #print(pos)
    pub_pos = rospy.Publisher('position', Position, queue_size=10)
    pub_vel = rospy.Publisher('velocity', Velocity, queue_size=10)
    pub_curr = rospy.Publisher('current', Current, queue_size=10)
    rospy.init_node('read_pos_vel_curr', anonymous=True)
    #set frequency of 10 Hz
    ros_freq = 10
    rate = rospy.Rate(ros_freq)
    while not rospy.is_shutdown():
        #read current and position
        read_pos_vel_curr(groupBulkRead, packetHandler, id, pos, vel, curr)
        print("ID: %03d, Position: %04d, Velocity: %0.4f, Current: %04d "%(pos.id, pos.position, vel.velocity, curr.current))
        pub_pos.publish(pos)
        pub_vel.publish(vel)
        pub_curr.publish(curr)
        #sleep for enough time to match frequency of 10 Hz
        rate.sleep()

def main():
    open_port(portHandler)
    set_baudrate(portHandler, BAUDRATE)
    # setOpMode(portHandler, packetHandler, DXL_ID, POSITION_CONTROL_MODE)
    # enable_torque(portHandler, packetHandler, DXL_ID)
    add_paramBR(groupBulkRead, DXL_ID, ADDR_PRESENT_CURR, LEN_PRESENT_DATA)
    # disable_torque(portHandler, packetHandler, DXL_ID)
    try:
        pos = Position()
        vel = Velocity()
        curr = Current()
        pub_pos_vel_curr(packetHandler, groupBulkRead, DXL_ID, pos, vel, curr)
    except rospy.ROSInterruptException:
        print("ROS Node Terminated")
    disable_torque(portHandler, packetHandler, DXL_ID)
    close_port(portHandler,groupBulkRead)


if __name__=='__main__':
    main()