#!/usr/bin/env python3
import os
import rospy
from dynamixel_sdk import *
from ros_dynamixel.msg import *
from ros_dynamixel.vars import *
from ros_dynamixel.utils import *


portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)
groupBulkRead = GroupBulkRead(portHandler, packetHandler)

def enable_torque(portHandler, packetHandler, id):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % id)

def add_param(groupBulkRead, id, param_addr, param_len):
    # Add parameter storage for Dynamixel#1 present position
    dxl_addparam_result = groupBulkRead.addParam(id, param_addr, param_len)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkRead addparam failed" % id)
        quit()

def bulkRead(groupBulkRead, packetHandler):
    dxl_comm_result = groupBulkRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

def extractData(groupBulkRead, id, param_addr, param_len):
    # Check if groupbulkread data of Dynamixel#1 is available
    dxl_getdata_result = groupBulkRead.isAvailable(id, param_addr, param_len)
    if dxl_getdata_result != True:
        print("[ID:%03d] groupBulkRead getdata failed" % id)
        quit()
    dxl_data = groupBulkRead.getData(id, param_addr, param_len)

    return dxl_data

def read_posCurr(id, groupBulkRead):
    pub = rospy.Publisher('position_current', posCurr, queue_size=10)
    rospy.init_node('read_posCurr', anonymous=True)
    rate = rospy.Rate(10)
    poscurr = posCurr()
    while not rospy.is_shutdown():
        poscurr.id = id
        bulkRead(groupBulkRead, packetHandler)
        poscurr.position = extractData(groupBulkRead, id, ADDR_PRESENT_POS, LEN_PRESENT_POS)
        poscurr.current = extractData(groupBulkRead, id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT)
        poscurr.time = int(rospy.get_time())
        print("ID: %03d, Position: %04d, Current: %05d, Time: %d"%(poscurr.id, poscurr.position, poscurr.current, poscurr.time))
        rate.sleep()

def main():
    open_port(portHandler)
    set_baudrate(portHandler, BAUDRATE)
    enable_torque(portHandler, packetHandler, DXL_ID)
    add_param(groupBulkRead, DXL_ID, ADDR_PRESENT_CURRENT, LEN_PRESENT_DATA)
    try:
        read_posCurr(DXL_ID, groupBulkRead)
    except rospy.ROSInterruptException:
        pass


if __name__=='__main__':
    main()