#!/usr/bin/env python2

import rospy
from sensor_model import SensorModel
from motion_model import MotionModel
from geometry_msgs.msgs import PoseWithCovarianceStamped
from nav_msgs.msgs import Odometry

class ParticleFilter:

    def __init__(self):

        # Get parameters
        self.particle_filter_frame = \
                rospy.get_param("~particle_filter_frame")
        self.ODOMETRY_TOPIC = "/odom"
        self.SCAN_TOPIC = "/scan"
        self.POSE_TOPIC = "/initialpose"
        #Set size of partcles: First number is #particles
        self.particles = np.zeros((3, 3))
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
        pass

    def scan_callback(self, scan):
        '''
        Take in scan. 
        Pass most recent partcles into sensor_model.
        Sample these particles given the last distribution.
        '''
        probs_for_particles = SensorModel.evaluate(self.particles, scan)
        new_particles = np.random.choice(self.particles, probs_for_particles)
        self.particles = new_particles

    def particle_setup(self, position):
        '''
        Take in position sent by "2D Pose Estimation"
        Set all particles to that pose.
        '''
        x, y = position.pose.pose.point.x, position.pose.pose.point.y
        theta = 2*np.atan(position.pose.pose.orientation.z/position.pose.pose.orientation.w)
        #Note on theta: This is calculated so 0 rad is pointing down on the map (along
        #grid marks).  Counterclockwise in + angle to pi and clockwise is - angle to pi
        self.particles[:] = np.array([x, y theta])


if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
