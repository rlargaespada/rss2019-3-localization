#!/usr/bin/env python2
import numpy as np
import rospy
from sensor_model import SensorModel
from motion_model import MotionModel
from geometry_msgs.msgs import PoseWithCovarianceStamped
from nav_msgs.msgs import Odometry, LaserScan 

class ParticleFilter:

    def __init__(self):

        # Get parameters
        self.particle_filter_frame = \
                rospy.get_param("~particle_filter_frame")
        self.sim = True
        if self.sim: 
            self.ODOMETRY_TOPIC = "/odom"
            self.SCAN_TOPIC = "/scan"
            self.POSE_TOPIC = "/initialpose"
            self.AVG_POSE_TOPIC = "/base_link_pf"
        else:
            self.ODOMETRY_TOPIC = "vesc/odom"
            self.SCAN_TOPIC = "/scan"
            self.POSE_TOPIC = "will fix later"
            self.AVG_POSE_TOPIC = "/base_link"
        #Set size of partcles: First number is #particles
        self.particles = np.zeros((3, 3))
        self.std_dev = 1 #standard deviation of simulated sensor noise
        # Initialize the models
        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()

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

    def odometry_callback(self, odometry):
        '''
        Take in odometry data.  
        Add noise via motion_model.
        '''
        #[dx, dy, dtheta]
        vel = np.zeros(3, 1)
        vel[0] = odometry.twist.twist.linear.x
        vel[1] = odometry.twist.twist.linear.y
        vel[2] = odometry.twist.twist.angular.z
        self.particles = self.motion_model.evaluate(self.particles, vel)
        self.get_avg_pose()

    def scan_callback(self, scan):
        '''
        Take in scan. 
        Pass most recent partcles into sensor_model.
        Sample these particles given the last distribution.
        '''
        probs_for_particles = self.sensor_model.evaluate(self.particles, scan)
        new_particles = np.random.choice(self.particles, probs_for_particles)
        self.particles = new_particles
        self.get_avg_pose()

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
        self.particles[:, 0] = x + np.random.randn(N)*self.std_dev
        self.particles[:, 1] = y + np.random.randn(N)*self.std_dev
        self.particles[:, 2] = theta + np.random.randn(N)*self.std_dev
        self.particles[:, 2] %= 2* np.pi

    def get_avg_pose(self):
        '''
        Gets the average pose from all the particles, saves to self and publishes.
        '''
        x_avg = np.average(self.particles[:,0])
        y_avg = np.average(self.particles[:,1])
        theta_avg = np.average(self.particles[:,2]) #fix
        #Publish this pose as a transformation between the /map frame and a frame for the expected car's base link.

if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
