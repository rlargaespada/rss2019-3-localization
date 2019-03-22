import numpy as np
from scan_simulator_2d import PyScanSimulator2D
import sensor_lookup
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler

class SensorModel:


    def __init__(self):

        # Fetch parameters
        self.map_topic = rospy.get_param("~map_topic")
        self.take_nth_beam = rospy.get_param("~take_nth_beams")
        self.num_beams_per_particle = rospy.get_param("~num_beams_per_particle")/self.take_nth_beam
        self.scan_theta_discretization = rospy.get_param("~scan_theta_discretization")
        self.scan_field_of_view = rospy.get_param("~scan_field_of_view")
        #meters/measurement in lookup table 
        self.grain = rospy.get_param("~lookup_grain")
        #Get lookup table: Values listed as (a_hit, a_short, a_max, a_rand, sigma, max_range, dz)
        self.prob_lookup = sensor_lookup.SensorTable(.74, .07, .07, .12, .5, 10, self.grain)
        ####################################
        # TODO
        # Precompute the sensor model here
        # (You should probably write a
        #  function for this)

        ####################################

        # Create a simulated laser scan
        self.scan_sim = PyScanSimulator2D(
                self.num_beams_per_particle,
                self.scan_field_of_view,
                0, # This is not the simulator, don't add noise
                0.01, # This is used as an epsilon
                self.scan_theta_discretization) 

        # Subscribe to the map
        self.map_set = False
        rospy.Subscriber(
                self.map_topic,
                OccupancyGrid,
                self.map_callback,
                queue_size=1)

    def evaluate(self, particles, observations):
        """
        Evaluate how likely each particle is given
        the observed scan.

        args:
            particles: An Nx3 matrix of the form:
            
                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ] 

            observation: A vector of lidar data of
                length N

        returns:
           probabilities: A vector of length N representing
               the probability of each particle existing
               given the observation and the map.
        """

        if not self.map_set:
            return

        ####################################
        # TODO
        # Evaluate the sensor model here!
        #
        # You will probably want to use this function
        # to perform ray tracing from all the particles.
        # This produces a matrix of size N x num_beams_per_particle 
        scans = self.scan_sim.scan(particles)
        #Bound the distance of a scan to 9.9
        scans = np.clip(scans, 0, 9.9)
        #Return probabilities
        return self.scans_to_probs(scans, observations, self.grain)
        ####################################

    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        map_ = np.array(map_msg.data, np.double)/100.
        map_ = np.clip(map_, 0, 1)

        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = tf.transformations.euler_from_quaternion((
                origin_o.x,
                origin_o.y,
                origin_o.z,
                origin_o.w))
        origin = (origin_p.x, origin_p.y, origin_o[2])

        # Initialize a map with the laser scan
        self.scan_sim.set_map(
                map_,
                map_msg.info.height,
                map_msg.info.width,
                map_msg.info.resolution,
                origin,
                0.5) # Consider anything < 0.5 to be free

        # Make the map set
        self.map_set = True

        print("Map initialized")

    def scans_to_probs(self, scans, laser_scan, grain):
        '''
        take in scans, observations, and grain
        '''
        observations = np.array(laser_scan.ranges)[::self.take_nth_beam]
        observations = np.clip(observations, 0, 9.9)
        if observations.shape[0] != scans.shape[1]:
            print("WARNING: observations and ray cast sizes different")
        #Initialize a matrix of probabilities associated with each ray
        probs = np.zeros((scans.shape[0], scans.shape[1]))
        #Iterate through each beam on a scan
        for beam in xrange(scans.shape[1]):
            #set the ground based measurement for each beam
            measurement = int(observations[beam]/grain)
            #Iterate through each particle
            for particle in xrange(scans.shape[0]):
                #Get the corrosponding measurement/scan probability from lookup table
                ray_cast = int(scans[particle, beam]/grain)
                # print(particle, ray_cast, measurement)
                probs[particle, beam] = self.prob_lookup.probs[measurement, ray_cast]

        #Find the overall probability of each scan by averaging each ray probability
        probs_mean = np.mean(probs, axis=1)
        #Get normalization constant for distribution of probabilities across particles
        probs_sum = np.sum(probs_mean)
        return probs_mean/probs_sum




