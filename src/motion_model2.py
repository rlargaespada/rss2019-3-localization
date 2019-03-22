class MotionModel:

    def __init__(self):

        ####################################
        # TODO
        # Do any precomputation for the motion
        # model here.

        self.noise = [0,0,0] # idea is to have potential noise in every direction

        # TODO set the noise

        ####################################

    def evaluate(self, particles, odometry):
        """
        Update the particles to reflect probable
        future states given the odometry data.

        args:
            particles: An Nx3 matrix of the form:
            
                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            odometry: A 3-vector [dx dy dtheta]

        returns:
            particles: An updated matrix of the
                same size
        """
        
        rows, columns = particles.shape
        updated_particles = np.zeros(rows, columns)
        for i in range(rows):
            (x,y,theta) = particles[i]
            odom_adjust = self.apply_odom(theta, odometry)
            odom_adjust += noise
            updated_particles[i] = odom_adjust + particles[i]
        return updated_particles

    def apply_odom(self, theta, odom_data):
        rotation_matrix = np.array([np.cos(theta), -np.sin(theta), 0],[np.sin(theta), np.cos(theta), 0], [0,0,1])
        return np.matmul(rotation_matrix, odom_data)
