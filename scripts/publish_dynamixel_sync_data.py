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
#Initialize GroupSyncRead instace for Current, Position and Velocity
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_CURR, LEN_PRESENT_DATA)
DXL_IDS = [1,2]
time.sleep(0.5)



def syncRead_pos_vel_curr(groupSyncRead, DXL_IDS, pos, vel, curr):
    #initialize IDs
    init_ids(pos,vel,curr, DXL_IDS)
    #perform the syncRead
    syncRead(groupSyncRead, packetHandler)
    #check if data is available
    checkDataAvailable(groupSyncRead, DXL_IDS, ADDR_PRESENT_CURR, LEN_PRESENT_DATA)
    #extract data from the syncRead
    extractSyncReadData(groupSyncRead, DXL_IDS, pos, vel, curr)

    
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
        print("ID_mcpf:%03d, Position_mcpf:%03d, Velocity_mcpf:%03d, Current_mcpf:%03d" % (pos.id_mcpf, pos.pos_mcpf, vel.vel_mcpf, curr.curr_mcpf))
        print("ID_mcpa:%03d, Position_mcpa:%03d, Velocity_mcpa:%03d, Current_mcpa:%03d" % (pos.id_mcpa, pos.pos_mcpa, vel.vel_mcpa, curr.curr_mcpa))
        print("ID_pip:%03d, Position_pip:%03d, Velocity_pip:%03d, Current_pip:%03d" % (pos.id_pip, pos.pos_pip, vel.vel_pip, curr.curr_pip))
        print("ID_dip:%03d, Position_dip:%03d, Velocity_dip:%03d, Current_dip:%03d" % (pos.id_dip, pos.pos_dip, vel.vel_dip, curr.curr_dip))
        print("----------------------------------------------------------------------")
        #publish data
        pub_pos.publish(pos)
        pub_vel.publish(vel)
        pub_curr.publish(curr)
        #sleep for enough time to match the frequency of publishing
        rate.sleep()

def pingDynamixels(packetHandler, DXL_IDS):
    for id in DXL_IDS:
        model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, id)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            print("ID:%03d ping Failed" % id)
            print("Error:%s" % packetHandler.getRxPacketError(dxl_error))
            DXL_IDS.remove(id)
        elif dxl_comm_result == COMM_SUCCESS:
            print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (id, model_number))
    time.sleep(3)
    return DXL_IDS
    

def main():
    global portHandler, packetHandler, groupSyncRead, DXL_IDS
    open_port(portHandler)
    set_baudrate(portHandler, BAUDRATE)
    #check if the dynamixels with ids in DXL_IDS are connected
    DXL_IDS = pingDynamixels(packetHandler, DXL_IDS)
    #add IDs for sync read of same data from multiple dynamixels
    add_SyncReadIDs(groupSyncRead, DXL_IDS)
    #sync read data from multiple dynamixels
    try:
        pos = PositionSync()
        vel = VelocitySync()
        curr = CurrentSync()
        pub_sync_pos_vel_curr(packetHandler, groupSyncRead, DXL_IDS, pos, vel, curr)
    except rospy.ROSInterruptException:
        print("ROS Node Terminated")
    
    #close port
    close_port_SR(portHandler,groupSyncRead)


if __name__ == '__main__':
    main()