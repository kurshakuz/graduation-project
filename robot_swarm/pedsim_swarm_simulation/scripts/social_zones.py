#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from pedsim_msgs.msg import AgentGroups
from visualization_msgs.msg import Marker, MarkerArray

pub = rospy.Publisher('social_zone', Marker, queue_size=10)

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.groups[0].members)
    # rospy.loginfo(data.groups[0].center_of_mass.position)
    # hello_str = "yyy"
    stamp = data.header.stamp
    frame_id = "odom"
    avg_radius = 2.5
    point = data.groups[0].center_of_mass.position

    marker = Marker()
    marker.header.stamp = stamp
    marker.header.frame_id = frame_id
    marker.ns = "social_zone"
    marker.id = 0

    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.scale.x = avg_radius
    marker.scale.y = avg_radius
    marker.scale.z = 0.001

    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    marker.pose.position.x = point.x
    marker.pose.position.y = point.y
    marker.pose.position.z = -0.001

    pub.publish(marker)
    
def talker():
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('social_zones', anonymous=True)
    try:
        rospy.Subscriber("/pedsim_simulator/simulated_groups", AgentGroups, callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass