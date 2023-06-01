#!/usr/bin/env python3
import rospy
from dynamixel_sdk import *
from ros_dynamixel.msg import *
from ros_dynamixel.utils import *
from ros_dynamixel.vars import *
from ros_dynamixel.comms import *


portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)



def read_current(id):
    pub = rospy.Publisher('current', Current, queue_size=10)
    rospy.init_node('read_current', anonymous=True)
    rate = rospy.Rate(10)
    current = Current()
    while not rospy.is_shutdown():
        current.id = id
        cur = get_present_current(id)
        current.current = cur if check_limit(cur,CURRENT_LIMIT) else current.current
        current.time = int(rospy.get_time())
        print("Present Current of ID %s = %s" % (id,current.current))
        rate.sleep()


def main():
    configure_shell_inputs()
    open_port(portHandler)
    set_baudrate(portHandler, BAUDRATE)

    try:
        read_current(DXL_ID)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()