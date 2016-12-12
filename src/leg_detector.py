import rospy
from sensor_msgs.msg import LaserScan
from random import gauss, randint, uniform
from bisect import bisect_left
from time import clock
import math
from DetectLegsFromRaw import collectProcessRawData, convertXY, getThetaRPoints
import matplotlib.pyplot as plt

# TODO Amanda
# set up the parameters however it's easiest for you
def plot_particles(self, particles):
    plt.ion() #enable interactive plotting
    plt.figure(2)
    for particle in particles:
	x = particle[0]
	y = particle[1]
	theta, r = getThetaRPoints(x, y)
	plt.plot(theta, r, 'ro')
	plt.ylabel('height of beam in m')
    	plt.xlabel('angle of laser scan in rad')
    	plt.pause(0.1) #for the 10 seconds
	plt.gcf().clear() #clear plot for next time

def plot_legs_after_resample(legs):
    plt.ion() #enable interactive plotting
    plt.figure(3)
    for leg in legs:
	x = leg.x
	y = leg.y
	theta, r = getThetaRPoints(x, y)
	plt.plot(theta, r, 'bo')
	plt.ylabel('height of beam in m for legs')
    	plt.xlabel('angle of laser scan in rad for legs')
    	plt.pause(0.1) #for the 10 seconds
	plt.gcf().clear() #clear plot for next time

def plot_people(people):
    plt.ion() #enable interactive plotting
    plt.figure(4)
    for person in people:
	leg1 = person[0]
	leg2 = person[1]
	theta1, r1 = getThetaRPoints(leg1.x, leg1.y)
	theta2, r2 = getThetaRPoints(leg2.x, leg2.y)
	avgTheta = (theta1 + theta2) / 2
	avgHeight = (r1 + r2) / 2
	plt.plot(avgTheta, avgHeight, 'go')
	plt.ylabel('height of beam in m for people')
    	plt.xlabel('angle of laser scan in rad for people')
    	plt.pause(0.1) #for the 10 seconds
	plt.gcf().clear() #clear plot for next time
	


#to play the rosbag file
# rosbag play Second.bag,Third.bag
def normpdf(x, mean, var):
    denom = (2*math.pi*var)**.5
    num = math.exp(-(float(x)-float(mean))**2/(2*var))
    return num/denom

def calc_variance(data):
    mean = math.fsum(data) / len(data)
    var = math.fsum([(x - mean)**2 for x in data]) / (len(data) - 1)
    return var

# hypothesized_leg_location is a position tuple (x, y)
# known_leg is a leg object
#this will between 0 and 1
def calculate_leg_correlation_score(hypothesized_leg_location, known_leg):
    obs_x, obs_y = hypothesized_leg_location
    x_score = normpdf(obs_x, known_leg.x, known_leg.x_var)
    y_score = normpdf(obs_y, known_leg.y, known_leg.y_var)
    return x_score * y_score

def calculate_legs_to_person_correlation(leg1, leg2):
    #print "this leg x: " + str(leg1)
    #print "other leg x: " + str(leg2)
    return calculate_leg_correlation_score((leg1.x, leg1.y), leg2) * calculate_leg_correlation_score((leg2.x, leg2.y), leg1)

class people_detector:
    def __init__(self):
        # need to be within 10cm of previous position to keep
        self.LEG_CORRELATION_THRESH = 0.1

        # legs need to be withing 50cm to be a person
        self.PEOPLE_CORRELATION_THRESH = 0.5
        # array of leg objects
        self.legs = []

        # array of tuples in the form (leg, leg)
        # if the distance between the two legs of a person gets too large (1 meter?), we take it out of self.people and put both legs back in 		#self.legs
        self.people = []
    	rospy.init_node('laser_scan_listener', anonymous=True)
    	rospy.Subscriber("/scan", LaserScan, self.process_scan_message)

    # this is the callback that is used when retrieving the message

    def update_legs(self, observed_legs):
        legs_to_process = []
        for leg in self.legs:
            max_correlation = 0
            most_likely_obs = None
            most_likely_idx = None

            for j, obs_leg in enumerate(observed_legs):
                correlation = calculate_leg_correlation_score(obs_leg, leg)
                if correlation > max_correlation:
                    max_correlation = correlation # to compare
                    most_likely_obs = obs_leg     # to process
                    most_likely_idx = j           # to delete from observed_legs

            if most_likely_obs:
                dist = math.sqrt((most_likely_obs[0] - leg.x)**2 + (most_likely_obs[1] - leg.y)**2)
                if dist <= self.LEG_CORRELATION_THRESH:
                    legs_to_process.append((leg, most_likely_obs))
                    del observed_legs[most_likely_idx]

        for obs in observed_legs:
            legs_to_process.append((None, obs))

        self.legs = []
        for leg, obs in legs_to_process:
            if leg:
                leg.resample(obs[0], obs[1])
                self.legs.append(leg)
            else:
                self.legs.append(Leg(obs[0], obs[1]))

    def correlate_legs_to_people(self):
        indices_seen = set()
        people = []
        print "start correlating people"
        for i, leg in enumerate(self.legs):
            # find the index of the other leg that corresponds to it
            idx = None
            max_correlation_score = 0

            if i not in indices_seen:
                for j in range(i+1, len(self.legs)):
                    other_leg = self.legs[j]
                    print "this leg: " + str(leg)
                    print "other leg: " + str(other_leg.x)
                    score = calculate_legs_to_person_correlation(leg, other_leg)
                    if score > max_correlation_score and j not in indices_seen:
                        max_correlation_score = score
                        idx = j
                if idx:
                    dx = leg.x - self.legs[idx].x
                    dy = leg.y - self.legs[idx].y
                    dist = math.sqrt(dx**2 + dy**2)
                    print "distance between legs:"
                    print dist
                    if dist <= self.PEOPLE_CORRELATION_THRESH:
                        indices_seen.add(idx)
                        people.append((leg, self.legs[idx]))

        self.people = people

    def process_scan_message(self, msg):
        # 1. read in laser data
    	observed_legs = convertXY(collectProcessRawData(msg.ranges, msg.range_min, msg.range_max, msg.angle_min, msg.angle_max, msg.angle_increment))

        print "Legs before update: " + str(len(self.legs))
        self.update_legs(observed_legs)
        print "Legs after update: " + str(len(self.legs))

        self.correlate_legs_to_people()

        print "omg self.people: " + str(self.people)
        #print("Legs: " + str(self.legs))
        #print("People: " + str(self.people))


        # TODO: Insert leg plotting code here
            # read self.legs <list of Leg objects>
            # read self.people <List of Leg object tuples>
            # plot each in a separate plot

            # TODO functions to fill in
        #plot_particles(particles)
        plot_legs_after_resample(self.legs)
	#plot_people(self.people)


    # read raw laser scan data
    # return a list of percieved legs
    def read_laser_scan(self, msg):
        print "hello"
        pass

class Leg:
    # constructor
    # sets up a leg
    def __init__(self, x, y, var=0.3):
        # set up class constants
        self.NUM_PARTICLES = 200

        # [-5, 5] degrees in radian coordinates
        self.THETA_PROPAGATION_NOISE = 2 * math.pi * 5 / 360

        # [-0.2, 0.2] meter positional movement
        self.R_PROPAGATION_NOISE = 0.2

        # setup variance
        self.x_var = var
        self.y_var = var

        # position
        self.x = x
        self.y = y

        # velocity
        self.x_vel = 0
        self.y_vel = 0

        # preallocate particles
        self.particles = [(0, 0) for i in range(self.NUM_PARTICLES)]
        for i in range(self.NUM_PARTICLES):
            px = gauss(x, var)
            py = gauss(y, var)
            ptheta = uniform(0, 2 * math.pi)
            self.particles[i] = (px, py, ptheta)

        self.time = clock()

    # particle filter step
    # x and y laser measurements of leg are passed in as arguments
    def resample(self, x_detected, y_detected):
        # get difference in time between runs
        dt = clock() - self.time

        # get weights of particles
        sum_weights = 0
        weights = [0 for i in range(self.NUM_PARTICLES)]
        for i, particle in enumerate(self.particles):
            px, py, ptheta = particle
            wx = normpdf(x_detected, self.x, self.x_var)
            wy = normpdf(y_detected, self.y, self.y_var)
            w = wx*wy
            sum_weights += w
            weights[i] = sum_weights

        # resample particles and add noisy movement
        resampled_particles = [None for i in range(self.NUM_PARTICLES)]
        for i in range(self.NUM_PARTICLES):

            # sample random particle
            sample_weight = uniform(0, sum_weights)
            idx = bisect_left(weights, sample_weight)
            px, py, ptheta = self.particles[idx]

            # movement detected
            r_0 = math.sqrt((self.x_vel * dt)**2 + (self.y_vel * dt)**2)

            # movement plus noise
            r_noise = r_0 + uniform(-self.R_PROPAGATION_NOISE, self.R_PROPAGATION_NOISE)

            # add random rotation
            theta_noise = ptheta + uniform(-self.THETA_PROPAGATION_NOISE, self.THETA_PROPAGATION_NOISE)

            # add movement plus noise to position
            x_noise = px + r_noise * math.cos(theta_noise)
            y_noise = py + r_noise * math.sin(theta_noise)

            resampled_particles[i] = (x_noise, y_noise, theta_noise)

        self.particles = resampled_particles
        x_sum = math.fsum([p[0] for p in self.particles])
        y_sum = math.fsum([p[1] for p in self.particles])
        new_x = x_sum / self.NUM_PARTICLES
        new_y = y_sum / self.NUM_PARTICLES

        # update velocity
        self.x_vel = (new_x - self.x) / dt
        self.y_vel = (new_y - self.y) / dt

        # update position
        self.x = new_x
        self.y = new_y

        # update variance
        self.x_var = calc_variance([p[0] for p in self.particles])
        self.y_var = calc_variance([p[1] for p in self.particles])

        self.time = clock()

if __name__ == '__main__':
    # execution entry point
    pd = people_detector()
    rospy.spin()
