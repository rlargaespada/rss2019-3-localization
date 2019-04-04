#!/usr/bin/env python2
import numpy as np
import rospy
import time
import tf
from tf.transformations import quaternion_from_euler
from sensor_model import SensorModel
from motion_model import MotionModel
import tf2_ros
import tf2_msgs.msg
from geometry_msgs.msg import PoseWithCovarianceStamped, Point32, Point, PoseStamped, Pose, Twist, Vector3
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, PointCloud
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from std_msgs.msg import Float32


class ParticleFilter:

    def __init__(self):

        # Get parameters
        self.PATH_TOPIC = "/path"
        self.PATH_TOPIC_ODOM = "/path_odom"

        rospy.Subscriber(self.ODOM_FOR_PATH, Pose, self.odom_path_callback)
        rospy.Subscriber(self.POSE_FOR_PATH, Twist, self.pose_path_callback)


    def pose_path_callback(self, twist):
        position = np.zeros(3)
        position[0] = twist.linear.x
        position[1] = twist.linear.y
        position[2] = twist.angular.z
        self.draw_path(position, self.path, self.path_pub)

    def publish_current_pose(self):
        twist = Twist()
        twist.linear = Vector3()
        twist.linear.x = self.current_pose[0]
        twist.linear.y = self.current_pose[1]
        twist.linear.z = 0
        twist.angular = Vector3()
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self.current_pose[2]
        self.pose_for_path_pub.publish(twist)


    def draw_path(self, wanted_pose, path, pub):
        '''
        creates a path for drawing in rviz
        wanted_pose in [x, y, theta]
        '''
        header = Header()
        header.stamp = rospy.rostime.Time.now()
        header.frame_id = "/map"
        point = Point()
        point.x = wanted_pose[0]
        point.y = wanted_pose[1]
        point.z = 0
        orient = Quaternion()
        quat = quaternion_from_euler(0, 0, wanted_pose[2])
        orient.x = quat[0]
        orient.y = quat[1]
        orient.z = quat[2]
        orient.w = quat[3]
        pose = Pose()
        pose.position = point
        pose.orientation = orient
        pose_stamp = PoseStamped()
        pose_stamp.pose = pose
        pose_stamp.header = header
        path.poses.append(pose_stamp)
        path.header = pose_stamp.header
        pub.publish(path)

if __name__ == "__main__":
    rospy.init_node("path_maker")
    pf = ParticleFilter()
    rospy.spin()