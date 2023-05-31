#!/usr/bin/env python3
import os
import rospy
from dynamixel_sdk import *
from ros_dynamixel.msg import *

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


# Control table address
ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POS = 132

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 1                 # Dynamixel ID : 1
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

POSITION_LIMIT = 4096

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def get_present_pos(id):
    dxl_present_pos, dxl_comm_resut, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_PRESENT_POS)
    return dxl_present_pos

def check_limit(val, limit):
    if(val>limit):
        return 0
    if(val<=limit):
        return 1
    
def open_port(portHandler):
    # Open port
    try:
       portHandler.openPort()
       print("Succeeded to open the port")
    except:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

def set_baudrate(portHandler, baudrate):
    # Set port baudrate
    try:
        portHandler.setBaudRate(baudrate)
        print("Succeeded to change the baudrate")
    except:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

def read_pos(id):
    pub = rospy.Publisher('position', Position, queue_size=10)
    rospy.init_node('read_pos', anonymous=True)
    rate = rospy.Rate(10)
    position = Position()
    while not rospy.is_shutdown():
        position.id = id
        pos = get_present_pos(id)
        position.position = pos if check_limit(pos, POSITION_LIMIT) else position.position
        position.time = int(rospy.get_time())
        print("Present Position of ID %s = %s" % (id,position.position))
        rate.sleep()

def main():
    open_port(portHandler)
    set_baudrate(portHandler, BAUDRATE)

    try:
        read_pos(DXL_ID)
    except rospy.ROSInterruptException:
        pass



if __name__ == '__main__':
    main()