#!usr/bin/env python3
import rospy
import os
from ros_dynamixel.vars import *
from ros_dynamixel.comms import *

def configure_shell_inputs():
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

def close_port(portHandler,groupBulkRead):
    # Open port
    try:
       clearBR(groupBulkRead)
       portHandler.closePort()
       print("Succeeded to close the port")
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

def calcVelProf(ros_freq, pos_inc):
    rev = pos_inc/POSITION_LIMIT
    rev_vel = rev*ros_freq
    vel_prof_max = rev_vel/REV_FACTOR
    return int(vel_prof_max)

