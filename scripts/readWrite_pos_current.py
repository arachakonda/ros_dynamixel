#!/usr/bin/env python3
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