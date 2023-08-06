import rospy
from dynamixel_sdk import *
from ros_dynamixel.msg import *
from ros_dynamixel.vars import *
from ros_dynamixel.utils import *
from ros_dynamixel.comms import *

def pub_dynamixel_control_current(current):
    pub = rospy.Publisher('currentControl', CurrentWrite, queue_size=10)
    rospy.init_node('control_dynamixel_current_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        current.id = 1
        current.current = 10
        print(current)
        pub.publish(current)
        rate.sleep()

def main():
    try:
        current = CurrentWrite()
        pub_dynamixel_control_current(current)
    except rospy.ROSInterruptException:
        print("ROS Node Terminated")

if __name__ == '__main__':
    main()
