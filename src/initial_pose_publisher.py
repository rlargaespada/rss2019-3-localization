#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

def talker():
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.init_node('initial_pose_publisher', anonymous=True)
    header = Header()
    pose_stamped = PoseWithCovarianceStamped()
    header.stamp = rospy.rostime.Time.now()
    header.frame_id = "/map"
    point = Point()
    point.x = -27
    point.y = -.5
    point.z = 0
    orient = Quaternion()
    orient.x = 0
    orient.y = 0
    orient.z = 00.01
    orient.w = -0.99
    pose = Pose()
    pose.position = point
    pose.orientation = orient
    pose_w_co = PoseWithCovariance()
    pose_w_co.pose = pose
    pose_w_co.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    pose_stamped.pose = pose_w_co
    pose_stamped.header = header
    pub.publish(pose_stamped)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass