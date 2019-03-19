import numpy as np

class SensorModel:

    def __init__(self, a_hit, a_short, a_max, a_rand, sigma, z_max, z_crit):
        self.a_hit = a_hit
        self.a_short = a_short
        self.a_max = a_max
        self.a_rand = a_rand
        self.z_max = z_max # maximum range of the scan
        self.z_crit = z_crit # ground truth from ray casting on map / mean of hit distribution
        self.sigma = sigma # std of hit distribution
        self.nu = 1

    def p_hit(self, zt):
        if 0 <= zt <= self.z_max:
            return self.nu * (np.sqrt(2*np.pi*self.sigma**2))**-1 * np.exp(-(zt-self.z_crit)**2/(2*self.sigma**2))

    def p_short(self, zt):
        if 0 <= zt <= self.z_crit:
            return (2/self.z_crit) * (1 - zt/self.z_crit)
        else:
            return 0

    def p_max(self, zt):
        if zt == self.z_max:
            return 1
        else:
            return 0

    def p_rand(self, zt):
        if 0 <= zt <= self.z_max:
            return 1/self.z_max
        else:
            return 0

    def measurement_prob(self, zt):
        return self.a_hit * self.p_hit(zt) + self.a_short * self.p_short(zt) + self.a_max * self.p_max(zt) + self.a_rand * self.p_rand(zt)


class SensorTable:

    def __init__(self, a_hit, a_short, a_max, a_rand, sigma, max_range, dz):
        self.a_hit = a_hit
        self.a_short = a_short
        self.a_max = a_max
        self.a_rand = a_rand
        self.z_max = max_range
        self.sigma = sigma # std of hit distribution
        self.dz = dz # granularity of lookup table
        self.nu = 1

        self.probs = np.zeros((int(max_range/dz), int(max_range/dz))) # index with [z_map, z_t]
        self.make_table()

    def p_hit(self):
        for z_map in xrange(0, int(self.z_max/self.dz)):
            for zt in xrange(0, int(self.z_max/self.dz)):
                self.probs[z_map, zt] += self.a_hit * self.nu *(np.sqrt(2*np.pi*self.sigma**2))**-1 *np.exp(-(zt*self.dz-z_map*self.dz)**2/(2*self.sigma**2))

    def p_short(self):
        for z_map in xrange(1, int(self.z_max/self.dz)):
            for zt in xrange(0, int(self.z_max/self.dz)):
                if zt <= z_map:
                    # print("here")
                    self.probs[z_map, zt] += self.a_short * (2./(z_map*self.dz)) * (1 - zt*1./z_map)
                    # print(self.a_short * (2./(z_map*self.dz)) * (1 - zt*1./z_map))
                else:
                    break

    def p_max(self):
        self.probs[:,-1] += self.a_max * 1


    def p_rand(self):
        self.probs += self.a_rand/self.z_max

    def make_table(self):
        self.p_hit()
        # print(self.probs[0,700])
        self.p_short()
        # print(self.probs[0, 700])
        self.p_max()
        # print(self.probs[0, 700])
        self.p_rand()
        # print(self.probs[0, 700])

# table = SensorTable(.74, .07, .07, .12, .5, 10, .01)
# print(table.probs[700, 800])