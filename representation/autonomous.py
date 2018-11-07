from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import numpy as np
import operator
import math

try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci

class Autonomous:

    def __init__(self, carID, framework, trajectory):
        self.carID = carID
        self.framework = framework
        self.trajectory = trajectory

    def getDistance(self, position1, position2):
        x1, y1 = position1
        x2, y2 = position2
        #try:
        dist = traci.simulation.getDistance2D(x1, y1, x2, y2, isDriving=False)
        # except:
        #     dist = -1
        return dist


    def getAngleBetweenCoordinates(self, p1, p2):
        xDiff = p2[0] - p1[0]
        yDiff = p2[1] - p1[1]
        # full_angle = np.absolute(math.degrees(math.atan2(yDiff, xDiff))) + 90
        full_angle = np.absolute(np.negative(math.degrees(math.atan2(yDiff, xDiff))) + 90)
        return full_angle


    def getDistanceToIntercept(self):

        distances_to_intercept = []
        ego_position = traci.vehicle.getPosition(self.carID)
        ego_angle = traci.vehicle.getAngle(self.carID)
        #print(self.framework)
        for i in range(len(self.framework)):
            interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other, inc, link, outg, index = self.framework[i]
            coordinate_angle = self.getAngleBetweenCoordinates(ego_position, self.framework[i][0])

            relative_angle = np.absolute(ego_angle - coordinate_angle)
            if relative_angle < 135:
                #print(ego_position, self.framework[i][0])
                #self.getDistance(ego_position, self.framework[i][0])
                distances_to_intercept.append((self.getDistance(ego_position, self.framework[i][0]), interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other, inc, link, outg, index))

        return sorted(distances_to_intercept, key=lambda x: x[0])


    def getTimesToIntersection(self, distances):

        ego_speed = np.maximum(traci.vehicle.getSpeed(self.carID), 0.001)

        times_to_intersection = []
        for i in range(len(distances)):
            dist, interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other, inc, link, outg, index = distances[i]
            #print(dist, ego_speed)
            tti = dist / ego_speed
            times_to_intersection.append((tti, dist, interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other, inc, link, outg, index))
        return times_to_intersection














