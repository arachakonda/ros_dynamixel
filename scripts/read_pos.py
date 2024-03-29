#!/usr/bin/env python3
import os
import rospy
from dynamixel_sdk import *
from ros_dynamixel.msg import *
from ros_dynamixel.utils import *
from ros_dynamixel.vars import *
from ros_dynamixel.comms import *

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def read_pos(id):
    pub = rospy.Publisher('position', Position, queue_size=10)
    rospy.init_node('read_pos', anonymous=True)
    rate = rospy.Rate(10)
    position = Position() #message type
    while not rospy.is_shutdown():
        position.id = id
        pos = get_present_pos(id, portHandler, packetHandler)
        position.position = pos if check_limit(pos, POSITION_LIMIT) else position.position
        position.time = int(rospy.get_time())
        print("Present Position of ID %s = %s" % (id,position.position))
        pub.publish(position)
        rate.sleep()

def main():
    open_port(portHandler)
    set_baudrate(portHandler, BAUDRATE)
    setOpMode(portHandler, packetHandler, DXL_ID, POSITION_CONTROL_MODE)
    enable_torque(portHandler, packetHandler, DXL_ID)
    disable_torque(portHandler, packetHandler, DXL_ID)
    try:
        read_pos(DXL_ID)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()