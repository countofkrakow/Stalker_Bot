
from random import gauss
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
            # TODO: Sample particle with x, y mean and given variance from a gaussian
            # particle = ...
            px = gauss(x, var)
            py = gauss(y, var)
            self.particles[i] = (px, py)

    # particle filter step
    # x and y laser measurements of leg are passed in as arguments
    # update self.(x, y, dx, dy).  Also update variance based on the particles samples in this step
    def resample(self, x_detected, y_detected):
        weights = []
        for i, particle in enumerate(particles):
            # TODO:
            # assign weights to particles
            # x, y = particle
            # x_weight = sample_from_gaussian(x_detected, x, cls.DEFAULT_VARIANCE)
            # ...
            pass

if __name__ == '__main__':
    # execution entry point
    pd = people_detector()
