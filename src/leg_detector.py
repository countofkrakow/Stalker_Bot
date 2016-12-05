
from random import gauss, randint, uniform
import math

def normpdf(x, mean, var):
    denom = (2*math.pi*var)**.5
    num = math.exp(-(float(x)-float(mean))**2/(2*var))
    return num/denom

# NOTE: more functions can and probably should be added to improve modularity of code

class people_detector:
    def __init__(self):
        # array of leg objects
        self.legs = []

        # array of tuples in the form (leg, leg)
        # if the distance between the two legs of a person gets too large (1 meter?), we take it out of self.people and put both legs back in self.legs
        self.people = []

    # this is the callback that is used when retrieving the message
    def process_scan_message(self, msg):
        # 1. read in laser data
        # observed_legs = read_laser_scan(msg)

        # 2.
        legs_to_process = []
        # for position in observed_legs:
        #   max_prob = 0
        #   most_likely_leg = None
        #   for leg in self.legs + [leg for person in self.people for leg in person]:
        #       if distance between the leg and position is below some threshold AND the probability of it being that leg as sampled from the gaussian is larger than max_prob,
        #       update max_prob to the new higher probability, and set most_likely_leg to leg

        # now that we've gone through all our legs, we can clear out old legs and only keep what we've seen
        # self.legs = []

        #   if most_likely_leg:
        #       we have our leg.
                legs_to_process.append((most_likely_leg, position))
        #   else:
        #       add leg described by position as a new leg to self.legs


        # 3.
        #   for leg in legs_to_process:
        #       update the leg position accorsing to the position measurement and add it back to self.legs
        #       use the leg.resample function for this

        # 4.
        #  for leg in self.legs:
        #       do nested loop to find two closest legs that are at most some distance apart. Add them to self.people
        #       repeat until either there's no more legs to process or they're all far enough apart that we know that none of the legs left belong to the same person

        # 5. publish list of people positions to some topic...

    # read raw laser scan data
    # return a list of percieved legs
    def read_laser_scan(self, msg):
        print "hello"
        pass

class leg:
    # constructor
    # sets up a leg
    def __init__(self, x, y, var=0.3):
        # set up class constants
        self.NUM_PARTICLES = 100

        # placeholder: fill in with something else maybe
        self.PROPAGATION_NOISE = 1
        self.variance = var

        # position
        self.x = x
        self.y = y

        # velocity
        # update this at the resample step
        self.dx = 0
        self.dy = 0

        # preallocate particles
        self.particles = [(0, 0) for i in range(self.NUM_PARTICLES)]
        for i in range(self.NUM_PARTICLES):
            px = gauss(x, var)
            py = gauss(y, var)
            self.particles[i] = (px, py)

    # particle filter step
    # x and y laser measurements of leg are passed in as arguments
    # update self.(x, y, dx, dy).  Also update variance based on the particles samples in this step
    def resample(self, x_detected, y_detected):

        # TODO: add movements sampling

        # get weights of particles
        sum_weights = 0
        weights = [0 for i in range(self.NUM_PARTICLES)]
        for i, particle in enumerate(self.particles):
            px, py = particle
            wx = normpdf(x_detected, self.x, self.variance)
            wy = normpdf(y_detected, self.y, self.variance)
            w = wx*wy
            sum_weights += w
            weights[i] = sum_weights

        # resample particles
        resampled_particles = [(0, 0) for i in range(self.NUM_PARTICLES)]
        for i in range(self.NUM_PARTICLES):
            # sample random weight
            r = uniform(0, sum_weights)
            j = 0
            while r > weights[j]:
                j += 1
            resampled_particles[i] = self.particles[j]

        self.particles = resampled_particles

        # get new x, dx and y, dy values
        x_sum = 0
        y_sum = 0
        for particle in self.particles:
            x, y = particle
            x_sum += x
            y_sum += y
        new_x = x_sum / self.NUM_PARTICLES
        new_y = y_sum / self.NUM_PARTICLES
        self.dx = new_x - self.x
        self.dy = new_y - self.y
        self.x = new_x
        self.y = new_y
        # TODO: update variance






if __name__ == '__main__':
    # execution entry point
    pd = people_detector()
