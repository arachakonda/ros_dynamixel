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

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)
groupBulkRead = GroupBulkRead(portHandler, packetHandler)

def read_pos_curr(groupBulkRead,packetHandler,id,poscurr):
    poscurr.id = id
    bulkRead(groupBulkRead, packetHandler)
    poscurr.position = extractData(groupBulkRead, id, ADDR_PRESENT_POS, LEN_PRESENT_POS)
    poscurr.current = extractData(groupBulkRead, id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT)
    poscurr.time = int(rospy.get_time())

def home_pos(portHandler, packetHandler, id, poscurr):
    setProfVel(portHandler, packetHandler, id, MAX_VEL_PROF_VAL)
    add_paramBW(groupBulkWrite, id, ADDR_GOAL_POS, LEN_GOAL_POS, 0)
    bulkWrite(groupBulkWrite, packetHandler)
    clearBW(groupBulkWrite)
    while poscurr.position != 0:
        read_pos_curr(groupBulkRead,packetHandler,id, poscurr)
        pass

def write_pos(groupBulkWrite, packetHandler, id, position):
    add_paramBW(groupBulkWrite, id, ADDR_GOAL_POS, LEN_GOAL_POS, position)
    bulkWrite(groupBulkWrite, packetHandler)
    clearBW(groupBulkWrite)


def pub_posCurr(portHandler, packetHandler, groupBulkRead, groupBulkWrite, id, poscurr):
    pub = rospy.Publisher('position_current', posCurr, queue_size=10)
    rospy.init_node('read_posCurr', anonymous=True)
    #set frequency of 10 Hz
    ros_freq = 10
    rate = rospy.Rate(ros_freq)
    position = 0
    pos_inc = 100
    while not rospy.is_shutdown():
        #read current and position
        read_pos_curr(groupBulkRead, packetHandler,id,poscurr)
        print("ID: %03d, Position: %04d, Current: %05d, Time: %d"%(poscurr.id, poscurr.position, poscurr.current, poscurr.time))
        #write position
        if (poscurr.position < POSITION_LIMIT-pos_inc):
            position+=pos_inc
            vel_prof_max = calcVelProf(ros_freq, pos_inc)
            setProfVel(portHandler, packetHandler, DXL_ID, vel_prof_max)
        else:
            position=0
            home_pos(portHandler,packetHandler,id,poscurr)

        write_pos(groupBulkWrite, packetHandler, id, position)
        #sleep for enough time to match frequency of 10 Hz
        rate.sleep()
    

def main():
    open_port(portHandler)
    set_baudrate(portHandler, BAUDRATE)
    setOpMode(portHandler, packetHandler, DXL_ID, POSITION_CONTROL_MODE)
    enable_torque(portHandler, packetHandler, DXL_ID)
    add_paramBR(groupBulkRead, DXL_ID, ADDR_PRESENT_CURRENT, LEN_PRESENT_DATA)
    try:
        poscurr = posCurr()
        home_pos(portHandler,packetHandler,DXL_ID,poscurr)
        pub_posCurr(portHandler, packetHandler, groupBulkRead, groupBulkWrite, DXL_ID, poscurr)
    except rospy.ROSInterruptException:
        close_port(portHandler,groupBulkRead)
    close_port(portHandler,groupBulkRead)


if __name__=='__main__':
    main()