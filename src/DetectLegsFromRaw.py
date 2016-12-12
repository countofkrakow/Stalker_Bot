#!/usr/bin/env python
'''
Created on Dec 4, 2016

@author: Amanda
'''
from numpy import inf, NAN
import numpy as np
from numpy import nan
import math
from math import sin, sqrt
import matplotlib.pyplot as plt

#takes in legs in form (theta, r)
def convertXY(legs):
    points = []
    for leg_theta, leg_r in legs:
    	points.append(getPoints(leg_theta, leg_r))
    return points


def getPoints(theta, r):
    x = math.cos(theta) * r
    y = math.sin(theta) * r
    return (x,y)

def euclidean(x1, y1, x2, y2):
    return sqrt((x2-x1)**2 + (y2-y1)**2)

def detectLegs(points, min_leg_width, max_leg_width, drop_delta,  ranges, angle_min, angle_max, angle_increment, range_min, range_max):
    detecting_legs = False
    legs = []
    countInd = 1; #start at index 1
    leg_drop = 0
    leg_start = None
    for point in points[1:]:
        theta, r = point
        prev_theta, prev_r = points[countInd - 1]
        if (prev_r == inf and r != inf):
            dist = -2*drop_delta
            prev_r = r
        elif prev_r != inf and r == inf:
            dist = 2*drop_delta

        else:
            dist = r - prev_r


        if (dist < (-1 * drop_delta)):
            leg_drop = dist #guaranteed neg value
            detecting_legs = True
            plt.plot(theta, r, 'go')
            leg_start = (theta, r)
        elif (dist > drop_delta and detecting_legs):
            detecting_legs = False
            #print "+"
            leg_end = (prev_theta, prev_r)
            pointStart = getPoints(leg_start[0], leg_start[1])
            pointEnd = getPoints(leg_end[0], leg_end[1])
            if pointStart and pointEnd:
                distPoints = euclidean(pointStart[0], pointStart[1], pointEnd[0], pointEnd[1])
                if (distPoints >= min_leg_width and distPoints <= max_leg_width):
                    mean_angle = (leg_start[0] + leg_end[0])/2
                    mean_height = (leg_start[1] + leg_end[1])/2
                    legs.append((mean_angle, mean_height))
        countInd = countInd + 1
    return legs

#generates legal points for the initial leg processor
#takes all nan points and makes them inf points
#takes in one frame from the laser scan
def generatePoints(ranges, range_min, range_max, angle_min, angle_max, angle_increment):
    count = 0;
    points = []
    for range in ranges:
        angle = angle_min + count * angle_increment
        if math.isnan(range):
            points.append((angle, inf))
        else:
            points.append((angle, range))
        count = count + 1
    return points

#graphs the data in real time
def graphData(points, legs):
    redPoints = []
    bluePoints = []
    for point in points:
        redPoint, = plt.plot(point[0], point[1], 'ro')
	redPoints.append(redPoint)
    for newP in legs:
        bluePoint, = plt.plot(newP[0], newP[1], 'bo')
	bluePoints.append(bluePoint)
    plt.ylabel('height of beam in m')
    plt.xlabel('angle of laser scan in rad')
    plt.pause(0.1) #for the 10 seconds
    # print "Size of legs: " + str(len(legs))
    '''
    #now we want to remove the point for the next iteration
    for redPoint in redPoints:
        redPoint.remove()
    for newP in bluePoints:
	print bluePoint
        bluePoint.remove()
    '''
    plt.gcf().clear()

def collectProcessRawData(ranges, range_min, range_max, angle_min, angle_max, angle_increment):
    plt.ion() #enable interactive plotting
    drop_delta = 0.1
    min_leg_width = 0.01
    max_leg_width = 0.3
    points = generatePoints(ranges, range_min, range_max, angle_min, angle_max, angle_increment)
    legs = detectLegs(points, min_leg_width, max_leg_width, drop_delta,  ranges, angle_min, angle_max, angle_increment, range_min, range_max)
    graphData(points, legs)
    return legs


if __name__ == '__main__':
	pass
