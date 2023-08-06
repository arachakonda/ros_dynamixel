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
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_CURR, LEN_GOAL_CURR)
#the order of the list is [mcpf, mcpa, pip, dip]
DXL_IDS = [1,2,5]
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

def print_data(pos, vel, curr):
    # print("ID_mcpf:%03d, Position_mcpf:%03d, Velocity_mcpf:%03d, Current_mcpf:%03d" % (pos.id_mcpf, pos.pos_mcpf, vel.vel_mcpf, curr.curr_mcpf))
    # print("ID_mcpa:%03d, Position_mcpa:%03d, Velocity_mcpa:%03d, Current_mcpa:%03d" % (pos.id_mcpa, pos.pos_mcpa, vel.vel_mcpa, curr.curr_mcpa))
    # print("ID_pip:%03d, Position_pip:%03d, Velocity_pip:%03d, Current_pip:%03d" % (pos.id_pip, pos.pos_pip, vel.vel_pip, curr.curr_pip))
    # print("ID_dip:%03d, Position_dip:%03d, Velocity_dip:%03d, Current_dip:%03d" % (pos.id_dip, pos.pos_dip, vel.vel_dip, curr.curr_dip))
    # print("----------------------------------------------------------------------")
    pass

    
def pub_sync_pos_vel_curr(packetHandler, groupSyncRead, DXL_IDS, pos, vel, curr):
    pub_pos = rospy.Publisher('positionSync', PositionSync, queue_size=10)
    pub_vel = rospy.Publisher('velocitySync', VelocitySync, queue_size=10)
    pub_curr = rospy.Publisher('currentSync', CurrentSync, queue_size=10)
    rospy.init_node('pubw_pos_vel_curr', anonymous=True)
    ros_freq = 10
    rate = rospy.Rate(ros_freq) # 10hz
    while not rospy.is_shutdown():
        #read data from dynamixels
        syncRead_pos_vel_curr(groupSyncRead, DXL_IDS, pos, vel, curr)
        print_data(pos, vel, curr)
        #publish data
        pub_pos.publish(pos)
        pub_vel.publish(vel)
        pub_curr.publish(curr)
        #sleep for enough time to match the frequency of publishing
        rate.sleep()

def pingDynamixels(packetHandler, DXL_IDS):
    pingable_ids = []
    for id in DXL_IDS:
        dxl_comm_result = COMM_TX_FAIL
        print("pinging ID:%03d" % id)
        model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, id)
        if dxl_comm_result != COMM_SUCCESS:
            print("[ID:%03d] ping Failed" % id)
        elif dxl_comm_result == COMM_SUCCESS:
            pingable_ids.append(id)
            print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (id, model_number))
        
    time.sleep(3)
    return pingable_ids

def write_currentCallback(data):
    #syncwrite into dynamixel

    # uint8_t dxl_error = 0;
    # int dxl_comm_result = COMM_TX_FAIL;
    # int dxl_addparam_result = false;
    # uint8_t param_goal_position1[4];
    # uint8_t param_goal_position2[4];

    # // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
    # uint32_t position1 = (unsigned int)msg->position1; // Convert int32 -> uint32
    # param_goal_position1[0] = DXL_LOBYTE(DXL_LOWORD(position1));
    # param_goal_position1[1] = DXL_HIBYTE(DXL_LOWORD(position1));
    # param_goal_position1[2] = DXL_LOBYTE(DXL_HIWORD(position1));
    # param_goal_position1[3] = DXL_HIBYTE(DXL_HIWORD(position1));
    # uint32_t position2 = (unsigned int)msg->position2; // Convert int32 -> uint32
    # param_goal_position2[0] = DXL_LOBYTE(DXL_LOWORD(position2));
    # param_goal_position2[1] = DXL_HIBYTE(DXL_LOWORD(position2));
    # param_goal_position2[2] = DXL_LOBYTE(DXL_HIWORD(position2));
    # param_goal_position2[3] = DXL_HIBYTE(DXL_HIWORD(position2));

    # // Write Goal Position (length : 4 bytes)
    # // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
    # dxl_addparam_result = groupSyncWrite.addParam((uint8_t)msg->id1, param_goal_position1);
    # if (dxl_addparam_result != true) {
    #     ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", msg->id1);
    # }

    # dxl_addparam_result = groupSyncWrite.addParam((uint8_t)msg->id2, param_goal_position2);
    # if (dxl_addparam_result != true) {
    #     ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", msg->id2);
    # }

    # dxl_comm_result = groupSyncWrite.txPacket();
    # if (dxl_comm_result == COMM_SUCCESS) {
    #     ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id1, msg->position1);
    #     ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id2, msg->position2);
    # } else {
    #     ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
    # }

    # groupSyncWrite.clearParam();

    dxl_error = 0
    dxl_comm_result = COMM_TX_FAIL
    dxl_addparam_result = False

    param_goal_current = [DXL_LOBYTE(data.current), DXL_HIBYTE(data.current)]
    print(param_goal_current)

    for id in DXL_IDS:
        dxl_addparam_result = groupSyncWrite.addParam(id, param_goal_current)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % id)
        else:
            print("[ID:%03d] groupSyncWrite addparam success" % id)
    
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("failed to set current")
    else:
        print("succeeded to set current")
    groupSyncWrite.clearParam()

    print(data)

def main():
    global portHandler, packetHandler, groupSyncRead, DXL_IDS
    open_port(portHandler)
    set_baudrate(portHandler, BAUDRATE)
    #check if the dynamixels with ids in DXL_IDS are connected
    DXL_IDS = pingDynamixels(packetHandler, DXL_IDS)
    for id in DXL_IDS:
        setOpMode(portHandler, packetHandler, id, CURRENT_CONTROL_MODE)
    #add IDs for sync read of same data from multiple dynamixels
    add_SyncReadIDs(groupSyncRead, DXL_IDS)
    #sync read data from multiple dynamixels
    for id in DXL_IDS:
        enable_torque(portHandler, packetHandler, id)
    try:
        pos = PositionSync()
        vel = VelocitySync()
        curr = CurrentSync()
        rospy.Subscriber('currentControl', CurrentWrite, write_currentCallback)
        pub_sync_pos_vel_curr(packetHandler, groupSyncRead, DXL_IDS, pos, vel, curr)
    except rospy.ROSInterruptException:
        print("ROS Node Terminated")
    
    for id in DXL_IDS:
        disable_torque(portHandler, packetHandler, id)
    #close port
    close_port_SR(portHandler,groupSyncRead)


if __name__ == '__main__':
    main()