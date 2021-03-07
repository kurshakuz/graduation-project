#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Float64

def publisher(leftvel,rightvel):

    left_front = rospy.Publisher('/joint1_torque_controller/command', Float64, queue_size=10)
    right_front = rospy.Publisher('/joint2_torque_controller/command', Float64, queue_size=10)
    left_rear = rospy.Publisher('/joint3_torque_controller/command', Float64, queue_size=10)
    right_rear = rospy.Publisher('/joint4_torque_controller/command', Float64, queue_size=10)

    rospy.init_node('jackal_cotrol_node')
    rate=rospy.Rate(10)

    left = Float64() 
    left.data = float(leftvel)
    right = Float64() 
    right.data = float(rightvel)

    while not rospy.is_shutdown():
        left_front.publish(left)
        right_front.publish(right)
        left_rear.publish(left)
        right_rear.publish(right)
        rate.sleep()

if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    if len(args) != 3:
        print "Not enough parameters"
        sys.exit(1)

    leftvel = args[1]
    rightvel = args[2]
    try:
        publisher(leftvel,rightvel)
    except rospy.ROSInterruptException:
        pass

