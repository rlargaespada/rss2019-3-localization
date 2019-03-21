import numpy as np

class MotionModel:

    def __init__(self, std_dev, delta_t):

        self.std_dev = std_dev #standard deviation of simulated sensor noise
        self.delta_t = delta_t #20Hz

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

        ####################################
        N = len(particles)
        #change heading
        particles[:, 2] += odometry[2]*self.delta_t + np.random.randn(N)*self.std_dev
        particles[:, 2] %= 2* np.pi

        #predict x and y positions
        particles[:, 0] += odometry[0]*self.delta_t + np.random.randn(N)*self.std_dev
        particles[:, 1] += odometry[1]*self.delta_t + np.random.randn(N)*self.std_dev

        return particles


        ####################################
