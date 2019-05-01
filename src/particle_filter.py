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
        self.debug = rospy.get_param("~debug")
        self.sim = rospy.get_param("~sim")

        # Get parameters
        self.debug = True
        self.sim = rospy.get_param("~sim")
        self.particle_filter_frame = rospy.get_param("~particle_filter_frame")
        self.ODOMETRY_TOPIC = rospy.get_param("~odom_topic")
        self.SCAN_TOPIC = rospy.get_param("~scan_topic")
        self.POSE_TOPIC = rospy.get_param("~pose_topic")
        self.AVG_POSE_TOPIC = rospy.get_param("~avg_pose_topic")
        self.PARTICLE_CLOUD_TOPIC = rospy.get_param("~particle_topic")
        self.VISUALIZATION_TOPIC = rospy.get_param("~vis_topic")
        self.NUM_PARTICLES = rospy.get_param("~num_particles")
        self.DRIVE_TOPIC = rospy.get_param("~drive_topic")

        self.POSE_ESTIM_TOPIC = "/estim_pose"

        self.ERROR_TOPIC = "/localize_error"
        self.PATH_TOPIC = "/path"
        self.PATH_TOPIC_ODOM = "/path_odom"
        self.ODOM_FOR_PATH = "/odom_for_path"
        self.POSE_FOR_PATH = "/pose_for_path"

        self.ERROR_TOPIC = "/localize_error"
        self.PATH_TOPIC = "/path"
        self.PATH_TOPIC_ODOM = "/path_odom"

        # Set size of partcles
        self.particles = np.zeros((self.NUM_PARTICLES, 3))
        self.odom_pose = np.zeros(3)
        # Get model parameters
        self.sensor_std = rospy.get_param("~sensor_std") # standard deviation of simulated sensor noise
        self.motion_std = rospy.get_param("~motion_std") # standard deviation of motion model noise
        self.dt = rospy.get_param("~dt") # timestep over which to apply actuation (seconds)
        # Initialize the models
        self.motion_model = MotionModel(self.motion_std, self.dt, self.NUM_PARTICLES)
        self.sensor_model = SensorModel()
        self.particle_cloud_publisher = rospy.Publisher(self.PARTICLE_CLOUD_TOPIC, PointCloud, queue_size=10)
        self.current_pose_publisher = rospy.Publisher(self.VISUALIZATION_TOPIC, Marker, queue_size=10)
        self.estim_pub = rospy.Publisher(self.POSE_ESTIM_TOPIC, Point32, queue_size=10)
        self.current_pose = np.zeros((3, 1))
        self.path = Path()
        self.path_odom = Path()

        #Initialize drive model
        self.steer_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.drive_msg = AckermannDriveStamped()
        # self.create_ackermann()

        #Initialize variables for path
        self.path = Path()
        self.path_odom = Path()
        self.path_pub = rospy.Publisher(self.PATH_TOPIC, Path, queue_size=10)
        self.path_odom_pub = rospy.Publisher(self.PATH_TOPIC_ODOM, Path, queue_size=10)
        self.pose_for_path_pub = rospy.Publisher(self.POSE_FOR_PATH, Twist, queue_size=10)
        self.odom_for_path_pub = rospy.Publisher(self.ODOM_FOR_PATH, Pose, queue_size=10)


        #Initialize Error Publisher
        self.error_pub = rospy.Publisher(self.ERROR_TOPIC, Point32, queue_size=10)
        self.error_msg = Point32()

        #Initialize map frame transforms
        self.transform_stamped_msg = TransformStamped()
        self.frame_transform_pub = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=10)
        self.br = tf.TransformBroadcaster()
        #Initialize time tracker
        self.time_last = time.time()

        # Implement the MCL algorithm
        # using the sensor model and the motion model
        #
        # Make sure you include some way to initialize
        # your particles, ideally with some sort
        # of interactive interface in rviz
        #
        # Publish a transformation frame between the map
        # and the particle_filter_frame.
        rospy.Subscriber(self.POSE_TOPIC, PoseWithCovarianceStamped, self.particle_setup)
        rospy.Subscriber(self.ODOMETRY_TOPIC, Odometry, self.odometry_callback)
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan_callback)
        # rospy.Subscriber(self.ODOM_FOR_PATH, Pose, self.odom_path_callback)
        # rospy.Subscriber(self.POSE_FOR_PATH, Twist, self.pose_path_callback)

        # timing gate
        self.in_scan = False # only allow motion to run when we are not processing scan data
        self.in_motion = False # only allow scan to run when not in motion

    def odometry_callback(self, odometry):
        '''
        Take in odometry data.
        Add noise via motion_model.
        '''
        if not self.in_scan:
            #Get velocity from odometry message
            self.in_motion = True
            vel = np.zeros((3, 1))
            vel[0] = odometry.twist.twist.linear.x  #d_x
            vel[1] = odometry.twist.twist.linear.y  #d_y
            vel[2] = odometry.twist.twist.angular.z #d_theta
            #get position from odometry
            self.odom_pose[0] = odometry.pose.pose.position.x
            self.odom_pose[1] = odometry.pose.pose.position.y
            self.odom_pose[2] = 2*np.arctan(odometry.pose.pose.orientation.z/odometry.pose.pose.orientation.w)
            #Get change in time
            time_change = time.time() - self.time_last
            self.time_last = time.time()
            #update particles
            self.particles = self.motion_model.evaluate(self.particles, vel, time_change)
            #Average pose
            self.current_pose = self.get_avg_pose()
            #Show particles via rviz
            self.create_PointCloud()

            #Draw path
            # self.odom_for_path_pub.publish(odometry.pose.pose)
            # self.publish_current_pose()
            # publish ackermann message

            # self.steer_pub.publish(self.drive_msg)
            self.in_motion = False

    def scan_callback(self, scan):
        '''
        Take in scan.
        Pass most recent partcles into sensor_model.
        Sample these particles given the last distribution.
        '''
        if not self.in_motion:
            self.in_scan = True
            #Get probabilities for particles
            probs_for_particles = self.sensor_model.evaluate(self.particles, scan)
            #Get indexes for the particles based on probability distribution
            particle_index = np.random.choice(self.particles.shape[0], self.particles.shape[0], replace=True, p=probs_for_particles)
            #Get particles corrosponding to the indexes chosen
            self.particles = self.particles[particle_index]
            #Create point cloud for the particles
            self.current_pose = self.get_avg_pose()
            # print(self.current_pose)
            self.create_PointCloud()

            self.in_scan = False

    def particle_setup(self, position):
        '''
        Take in position sent by "2D Pose Estimation"
        Set all particles to that pose.
        '''
        x, y = position.pose.pose.position.x, position.pose.pose.position.y
        theta = 2*np.arctan(position.pose.pose.orientation.z/position.pose.pose.orientation.w)
        #Note on theta: This is calculated so 0 rad is pointing down on the map (along
        #grid marks).  Counterclockwise in + angle to pi and clockwise is - angle to pi
        N = len(self.particles)
        self.particles[:, 0] = x + np.random.randn(N)*self.sensor_std
        self.particles[:, 1] = y + np.random.randn(N)*self.sensor_std
        # self.particles[:, 2] = theta + np.random.randn(N)*self.sensor_std
        self.particles[:, 2] = np.random.uniform(0, 2*np.pi, N)
        # self.particles[:, 2] %= 2* np.pi
        self.time_last = time.time()
        self.path_odom = Path()
        self.path = Path()
        print(self.particles[1])

    def get_avg_pose(self):
        '''
        Gets the average pose from all the particles, saves to self and publishes.
        '''
        theta_avg = np.arctan2(np.average(np.sin(self.particles[:, 2])), np.average(np.cos(self.particles[:, 2])))
        x_avg = np.average(self.particles[:,0]) - .2*np.cos(theta_avg) #Convert to base_link
        y_avg = np.average(self.particles[:,1]) - .2*np.sin(theta_avg) #Convert to base_link
        avg = np.array([x_avg, y_avg, theta_avg])
        err = avg - self.odom_pose

        self.error_msg.x = err[0]
        self.error_msg.y = err[1]
        self.error_msg.z = err[2]

        if self.debug:
            self.error_pub.publish(self.error_msg)

        #how to handle multimodal avg?
        #Publish this pose as a transformation between the /map frame and a frame for the expected car's base link.
        return avg

    def multi_avg(self,vals):
        '''
        finds typical x value, robust to multi modal input
        '''
        x = np.array(vals)
        x_new = x[::5] + x[1::5] + x[2::5] + x[3::5] + x[4::5]
        xn = x_new.tolist()
        max1 = max(xn)
        m = max1
        xn.remove(max1)
        ms = [m]
        while m>=.8*max1:
            m = max(xn)
            xn.remove(m)
            ms.append(m)
        xn = x_new.tolist()
        if len(ms)>1:
            if abs(xn.index(max1)-xn.index(m[1]))>4:
                #dist is multi-modal
                True
            else:
                return np.average(vals)
        else:
            return np.average(vals)

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


    def create_PointCloud(self):
        '''
        Create and publish point cloud of particles and current pose marker
        '''
        cloud = PointCloud()
        cloud.header.frame_id = "/map"
        cloud.points = [Point32() for i in range(len(self.particles))]
        for point in range(len(cloud.points)):
            cloud.points[point].x = self.particles[point, 0]
            cloud.points[point].y = self.particles[point, 1]
            cloud.points[point].z = 0

        #arrow marker for current pose
        current_pose = Marker()
        current_pose.header.frame_id = "/map"
        current_pose.header.stamp = rospy.Time.now()
        current_pose.ns = "current_pose_marker"
        current_pose.id = 0
        current_pose.type = current_pose.ARROW
        current_pose.action = current_pose.ADD

        #start point and end point
        current_pose.points = [Point(), Point()]
        #start point
        current_pose.points[0].x = self.current_pose[0]
        current_pose.points[0].y = self.current_pose[1]
        current_pose.points[0].z = 0
        #end point
        current_pose.points[1].x = np.cos(self.current_pose[2]) + self.current_pose[0]
        current_pose.points[1].y = np.sin(self.current_pose[2]) + self.current_pose[1]
        current_pose.points[1].z = 0

        current_pose.scale.x = 0.2
        current_pose.scale.y = 0.4

        current_pose.color.a = 1.0
        current_pose.color.g = 1.0

        current_point = Point32()
        current_point.x = current_pose.points[0].x
        current_point.y = current_pose.points[0].y
        current_point.z = np.arctan2(current_pose.points[1].y - current_pose.points[0].y, current_pose.points[1].x - current_pose.points[0].x)

        self.particle_cloud_publisher.publish(cloud)
        self.current_pose_publisher.publish(current_pose)

        self.estim_pub.publish(current_point)
        if not self.sim:
            self.create_transform()
            self.br.sendTransform((self.current_pose[0], self.current_pose[1], 0), (self.transform_stamped_msg.transform.rotation.x, self.transform_stamped_msg.transform.rotation.y, self.transform_stamped_msg.transform.rotation.z, self.transform_stamped_msg.transform.rotation.w), rospy.Time.now(), "/base_link", "/map")
            tfm = tf2_msgs.msg.TFMessage([self.transform_stamped_msg])
            self.frame_transform_pub.publish(tfm)

    def create_ackermann(self):
        self.drive_msg.header.stamp = rospy.Time.now()
        self.drive_msg.header.frame_id = "1"
        self.drive_msg.drive.steering_angle = 0.
        self.drive_msg.drive.steering_angle_velocity = 0
        self.drive_msg.drive.speed = 1.
        self.drive_msg.drive.acceleration = 0
        self.drive_msg.drive.jerk = 0

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


if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
