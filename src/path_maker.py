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


class PathMaker:

    def __init__(self):

        # Get parameters
        self.sim = rospy.get_param("~sim")
        self.PATH_TOPIC = "/path"
        self.PATH_TOPIC_ODOM = "/path_odom"
        self.ODOM_FOR_PATH = "/odom_for_path"
        self.POSE_FOR_PATH = "/pose_for_path"
        self.POSE_TOPIC = rospy.get_param("~pose_topic")

        self.path = Path()
        self.path_odom = Path()
        self.last_initial_pose = np.zeros(3)

        self.path_pub = rospy.Publisher(self.PATH_TOPIC, Path, queue_size=10)
        self.path_odom_pub = rospy.Publisher(self.PATH_TOPIC_ODOM, Path, queue_size=10)

        rospy.Subscriber(self.POSE_TOPIC, PoseWithCovarianceStamped, self.path_reset)
        rospy.Subscriber(self.ODOM_FOR_PATH, Pose, self.odom_path_callback)
        rospy.Subscriber(self.POSE_FOR_PATH, Twist, self.pose_path_callback)

    def path_reset(self, position):
        x, y = position.pose.pose.position.x, position.pose.pose.position.y
        theta = 2*np.arctan(position.pose.pose.orientation.z/position.pose.pose.orientation.w)
        self.last_initial_pose = np.array([x, y, theta])
        self.path = Path()
        self.path_odom = Path()


    def odom_path_callback(self, pose):
        position = np.zeros(3)
        position[0] = pose.position.x
        position[1] = pose.position.y
        position[2] = 2*np.arctan(pose.orientation.z/pose.orientation.w)
        self.draw_path(position, self.path_odom, self.path_odom_pub)

    def pose_path_callback(self, twist):
        position = np.zeros(3)
        position[0] = twist.linear.x
        position[1] = twist.linear.y
        position[2] = twist.angular.z
        if not self.sim:
            position += self.last_initial_pose
        self.draw_path(position, self.path, self.path_pub)


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
    pm = PathMaker()
    rospy.spin()
