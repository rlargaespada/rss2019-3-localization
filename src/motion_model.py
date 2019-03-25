import numpy as np

class MotionModel:

    def __init__(self, std_dev, delta_t, num_particles):

        self.std_dev = std_dev #standard deviation of simulated sensor noise
        self.delta_t = delta_t #20Hz
        self.odom_adjust = np.zeros((num_particles, 3))          #Will become our new particles

    def evaluate(self, particles, odometry, delta_t):
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
        # N = len(particles)
        # #change heading
        # particles[:, 2] += odometry[2]*self.delta_t + np.random.randn(N)*self.std_dev
        # particles[:, 2] %= 2* np.pi

        # #predict x and y positions
        # particles[:, 0] += odometry[0]*self.delta_t + np.random.randn(N)*self.std_dev
        # particles[:, 1] += odometry[1]*self.delta_t + np.random.randn(N)*self.std_dev

        # return particles

        N = len(particles)                      # Number of particles
        odom_corrected = odometry*delta_t       #Translate the velocities into changes in x, y, and theta
        #Iterate through every particle
        for i in range(N):
            #Retrieve theta from particle
            theta = particles[i, 2]
            #Transform our changes into map reference frame
            self.odom_adjust[i, :] = self.apply_odom(theta, odom_corrected).T
        #print("particles", particles[:2, :])
        th = particles[:,2] # (200,3)
        p = particles #(200,3)
        o = odom_corrected #(3,1)
        v_apply_odom = np.vectorize(self.apply_odom2)
        self.odom_adjust2 = v_apply_odom(th,o,N)
        
        
        
        
        
        #r = np.array([[np.cos(th), -np.sin(th), np.zeros(len(th))],
        #              [np.sin(th), np.cos(th), np.zeros(len(th))], 
        #[np.zeros(len(th)),np.zeros(len(th)),1.0*np.ones(len(th))]])
        #r = r.reshape((3,3,1))
        #p = np.repeat(particles[:, :, np.newaxis], 3, axis=2)
        #p = np.swapaxes(p, 1, 2)
        
        #print('th', th.shape)
        #print('r ',r.shape)
        #print('particles ',p)
        #print("particles_shape ", p.shape)
        #m = np.matmul(p,r)
        #self.odom_adjust2 = np.zeros((200,3))
        #for i in range(200):
        #    self.odom_adjust2[i,:] = m[:,i,i]
            
        #print(self.odom_adjust2.shape)
        
        print('with for loop')
        print(self.odom_adjust)
        print('vectorized')
        print(self.odom_adjust2)

        #add noise to each dimension
        self.odom_adjust[:, 2] += np.random.randn(N)*self.std_dev*delta_t
        self.odom_adjust[:, 0] += np.random.randn(N)*self.std_dev*delta_t
        self.odom_adjust[:, 1] += np.random.randn(N)*self.std_dev*delta_t
        #return updated particles
        return particles + self.odom_adjust
    
    def apply_odom2(self, th, o, N):
        r = np.array([[np.cos(th), -np.sin(th), np.zeros(N)],
                      [np.sin(th), np.cos(th), np.zeros(N)], 
                      [np.zeros(N),np.zeros(N),1.0*np.ones(N)]])
        print(r.shape)
        print(o.shape)
        return np.matmul(r, o)   
        

    def apply_odom(self, theta, odom_data):
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta), 0],
                                    [np.sin(theta), np.cos(theta), 0], 
                                    [0,0,1.0]])
        return np.matmul(rotation_matrix, odom_data)

        ####################################
