import numpy as np
from scan_simulator_2d import PyScanSimulator2D
import sensor_lookup
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler
#from pathlib import Path

class SensorModel:

    def __init__(self):

        # Fetch parameters
        self.map_topic = rospy.get_param("~map_topic")
        self.take_nth_beam = rospy.get_param("~take_nth_beams")
        self.num_beams_per_particle = rospy.get_param("~num_beams_per_particle")/self.take_nth_beam
        self.scan_theta_discretization = rospy.get_param("~scan_theta_discretization")
        self.scan_field_of_view = rospy.get_param("~scan_field_of_view")
        self.scan_dist = rospy.get_param("~scan_dist")
        # meters/measurement in lookup table
        self.grain = rospy.get_param("~lookup_grain")
        self.sensor_std = rospy.get_param("~sensor_std")
        # Get lookup table: Values listed as (a_hit, a_short, a_max, a_rand, sigma, max_range, dz)
        self.params = np.array([.74, .07, .07, .12, self.sensor_std, self.scan_dist, self.grain])
        self.prob_lookup = sensor_lookup.SensorTable(.74, .07, .07, .12, self.sensor_std, self.scan_dist, self.grain)

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

        # This produces a matrix of size N x num_beams_per_particle
        scans = self.scan_sim.scan(particles)
        # Bound the distance of a scan to 9.9
        scans = np.clip(scans, 0, self.scan_dist - self.grain)
        # Return probabilities
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
        observations = np.clip(observations, 0, self.scan_dist - self.grain)
        observations = observations/self.grain
        observations = observations.astype(int)

        scans = scans/self.grain
        scans = scans.astype(int)

        obs_extended = np.tile(observations, (scans.shape[0], 1))
        probs = self.prob_lookup.probs[obs_extended, scans]

        # Find the overall probability of each scan by averaging each ray probability
        probs_mean = np.mean(probs, axis=1)
        # Get normalization constant for distribution of probabilities across particles
        probs_sum = np.sum(probs_mean)
        return probs_mean/probs_sum

#     def make_table(self):
#         dir_path = Path(__file__).parent

#         last_params = np.genfromtxt(str(dir_path) + '/lastparams.csv', delimiter=',')
#         if np.allclose(self.params, last_params):
#             self.prob_lookup = np.genfromtxt(str(dir_path) + '/SensorTable.csv', delimiter=',')
#         else:
#             np.savetxt('lastparams.csv', self.params, delimiter=',')
#             self.prob_lookup = sensor_lookup.SensorTable(self.params[0], self.params[1], self.params[2], self.params[3],
#             self.params[4], self.params[5], self.params[6])
