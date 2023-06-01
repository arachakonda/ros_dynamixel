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



def readwrite_posCurr(id, groupBulkRead):
    pub = rospy.Publisher('position_current', posCurr, queue_size=10)
    rospy.init_node('read_posCurr', anonymous=True)
    #set frequency of 10 Hz
    rate = rospy.Rate(10)
    poscurr = posCurr()
    position = 0
    while not rospy.is_shutdown():
        #read current and position
        poscurr.id = id
        bulkRead(groupBulkRead, packetHandler)
        poscurr.position = extractData(groupBulkRead, id, ADDR_PRESENT_POS, LEN_PRESENT_POS)
        poscurr.current = extractData(groupBulkRead, id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT)
        poscurr.time = int(rospy.get_time())
        print("ID: %03d, Position: %04d, Current: %05d, Time: %d"%(poscurr.id, poscurr.position, poscurr.current, poscurr.time))
        #write position
        if check_limit(position, POSITION_LIMIT) and (poscurr.position < POSITION_LIMIT-10):
            position+=100
        else:
            position=0
        add_paramBW(groupBulkWrite, DXL_ID, ADDR_GOAL_POS, LEN_GOAL_POS, position)
        bulkWrite(groupBulkWrite, packetHandler)
        clearBW(groupBulkWrite)
        #sleep for enough time to match frequency of 10 Hz
        rate.sleep()
    clearBR(groupBulkRead)

def main():
    open_port(portHandler)
    set_baudrate(portHandler, BAUDRATE)
    enable_torque(portHandler, packetHandler, DXL_ID)
    add_paramBR(groupBulkRead, DXL_ID, ADDR_PRESENT_CURRENT, LEN_PRESENT_DATA)
    try:
        readwrite_posCurr(DXL_ID, groupBulkRead)
    except rospy.ROSInterruptException:
        pass


if __name__=='__main__':
    main()