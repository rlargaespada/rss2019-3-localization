"""
    RSS 2019 | sensor_lookup.py
    Creates a lookup table for laser reading probabilities for a map of defined
    size.

    Authors: Abbie Lee (abbielee@mit.edu) and Alex Cuellar (acuel@mit.edu)

"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

class SensorTable:

    def __init__(self, a_hit, a_short, a_max, a_rand, sigma, max_range, dz):
        self.a_hit = a_hit
        self.a_short = a_short
        self.a_max = a_max
        self.a_rand = a_rand
        self.z_max = max_range
        self.sigma = sigma # std of hit distribution
        self.dz = dz # granularity of lookup table
        self.num_points = int(self.z_max/self.dz)

        self.probs = np.zeros((self.num_points, self.num_points)) # index with [z_map, z_t]
        self.make_table()

    def p_hit(self):
        # counter = 0
        # for z_map in np.linspace(0, self.z_max, int(self.z_max/self.dz)):
        #     zt = np.linspace(0, self.z_max, int(self.z_max/self.dz))
        #     new_row = self.a_hit *(np.sqrt(2*np.pi*self.sigma**2))**-1 * np.exp(-(zt-z_map)**2/(2*self.sigma**2))
        #     self.probs[counter,:] += new_row
        #     counter += 1
        for z_map in xrange(0, int(self.z_max/self.dz)):
            for zt in xrange(0, int(self.z_max/self.dz)):
                self.probs[z_map, zt] += self.a_hit *(np.sqrt(2*np.pi*self.sigma**2))**-1 * np.exp(-(self.dz*(zt-z_map))**2/(2*self.sigma**2))
        # self.normalize()

    def p_short(self):
        for z_map in xrange(1, int(self.z_max/self.dz)):
            for zt in xrange(0, int(self.z_max/self.dz)):
                if zt <= z_map:
                    self.probs[z_map, zt] += self.a_short * (2./(z_map*self.dz)) * (1 - zt*1./z_map)
                else:
                    break

    def p_max(self):
        self.probs[:,-1] += self.a_max * 1

    def p_rand(self):
        self.probs += self.a_rand/self.z_max

    def normalize(self):
        col_total = np.sum(self.probs, axis = 0)
        self.probs = self.probs / col_total

    def make_table(self):
        self.p_hit()
        # print(self.probs[700,0])
        self.p_short()
        # print(self.probs[700, 0])
        self.p_max()
        # print(self.probs[700, 0])
        self.p_rand()
        # print(self.probs[700, 0])

    def plot3d(self):
        fig = plt.figure()
        ax = fig.gca(projection='3d')

        # Make data
        x = np.linspace(0, self.z_max, int(self.z_max/self.dz))
        y = np.linspace(0, self.z_max, int(self.z_max/self.dz))
        x, y = np.meshgrid(x, y)
        z = self.probs

        surf = ax.plot_surface(x, y, z, cmap=cm.coolwarm, linewidth=0, antialiased=False)

        ax.set_xlabel("Measured")
        ax.set_ylabel("Ground Truth")
        ax.set_zlabel("Probability")
        ax.set_zlim(0, 0.8)

        plt.show()

    def plot2d(self):
        plt.plot(self.probs[700,:])
        plt.show()

# table = SensorTable(.74, .07, .07, .12, .5, 10, .01)
# # print(table.probs[700, 800])
# print(table.probs[700, 700])
# table.plot3d()
