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
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POS = 132

LEN_PRESENT_POS = 4
LEN_PRESENT_CURRENT = 2

LEN_PRESENT_DATA = 10

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

CURRENT_LIMIT = 3000


portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)
groupBulkRead = GroupBulkRead(portHandler, packetHandler)

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