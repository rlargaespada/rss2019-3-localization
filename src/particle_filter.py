#!/usr/bin/env python2
import numpy as np
import rospy
import time
from sensor_model import SensorModel
from motion_model import MotionModel
from geometry_msgs.msg import PoseWithCovarianceStamped, Point32, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped

class ParticleFilter:

    def __init__(self):

        # Get parameters
        self.particle_filter_frame = rospy.get_param("~particle_filter_frame")
        self.ODOMETRY_TOPIC = rospy.get_param("~odom_topic")
        self.SCAN_TOPIC = rospy.get_param("~scan_topic")
        self.POSE_TOPIC = rospy.get_param("~pose_topic")
        self.AVG_POSE_TOPIC = rospy.get_param("~avg_pose_topic")
        self.PARTICLE_CLOUD_TOPIC = rospy.get_param("~particle_topic")
        self.VISUALIZATION_TOPIC = rospy.get_param("~vis_topic")
        self.NUM_PARTICLES = rospy.get_param("~num_particles")
        self.DRIVE_TOPIC = rospy.get_param("~drive_topic")

        # Set size of partcles
        self.particles = np.zeros((self.NUM_PARTICLES, 3))

        # Get model parameters
        self.sensor_std = rospy.get_param("~sensor_std") # standard deviation of simulated sensor noise
        self.motion_std = rospy.get_param("~motion_std") # standard deviation of motion model noise
        self.dt = rospy.get_param("~dt") # timestep over which to apply actuation (seconds)
        # Initialize the models
        self.motion_model = MotionModel(self.motion_std, self.dt)
        self.sensor_model = SensorModel()
        self.particle_cloud_publisher = rospy.Publisher(self.PARTICLE_CLOUD_TOPIC, PointCloud, queue_size=10)
        self.current_pose_publisher = rospy.Publisher(self.VISUALIZATION_TOPIC, Marker, queue_size=10)
        self.current_pose = np.zeros((3, 1))

        #Initialize drive model
        self.steer_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.drive_msg = AckermannDriveStamped()
        self.create_ackermann()

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
            #Get change in time
            time_change = time.time() - self.time_last
            self.time_last = time.time()
            #update particles
            self.particles = self.motion_model.evaluate(self.particles, vel, time_change)
            #Average pose
            self.current_pose = self.get_avg_pose()
            #Show particles via rviz
            self.create_PointCloud()
            #publish ackermann message
            self.steer_pub.publish(self.drive_msg)
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
            self.particles = np.array([self.particles[i, :] for i in particle_index])
            #Create point cloud for the particles
            self.current_pose = self.get_avg_pose()
            print(self.current_pose)
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
        self.particles[:, 2] = theta + np.random.randn(N)*self.sensor_std
        self.particles[:, 2] %= 2* np.pi
        self.time_last = time.time()
        print(self.particles[1])

    def get_avg_pose(self):
        '''
        Gets the average pose from all the particles, saves to self and publishes.
        '''
        x_avg = np.average(self.particles[:,0])
        y_avg = np.average(self.particles[:,1])
        theta_avg = np.arctan2(np.average(np.sin(self.particles[:, 2])), np.average(np.cos(self.particles[:, 2])))
        avg = np.array([x_avg, y_avg, theta_avg])

        #how to handle multimodal avg?
        #Publish this pose as a transformation between the /map frame and a frame for the expected car's base link.
        return avg

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
        current_pose.points[1].y = np.cos(self.current_pose[2]) + self.current_pose[1]
        current_pose.points[1].z = 0

        current_pose.scale.x = 0.2
        current_pose.scale.y = 0.4

        current_pose.color.a = 1.0
        current_pose.color.g = 1.0

        self.particle_cloud_publisher.publish(cloud)
        self.current_pose_publisher.publish(current_pose)

    def create_ackermann(self):
        self.drive_msg.header.stamp = rospy.Time.now()
        self.drive_msg.header.frame_id = "1"
        self.drive_msg.drive.steering_angle = 0.
        self.drive_msg.drive.steering_angle_velocity = 0
        self.drive_msg.drive.speed = 1.
        self.drive_msg.drive.acceleration = 0
        self.drive_msg.drive.jerk = 0





if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
