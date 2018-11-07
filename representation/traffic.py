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

class Traffic:

    def __init__(self, distances):
        self.distances = distances


    def getDistance(self, position1, position2):
        x1, y1 = position1
        x2, y2 = position2
        #print(position1, position2)
        #print(x1, y1, x2, y2)
        dist = traci.simulation.getDistance2D(x1, y1, x2, y2, isDriving=False)
        #dist2 = traci.simulation.getDistance2D(x1, y1, x2, y2, isDriving=True)
        #print(dist, dist2)
        return dist



    def getAngleBetweenCoordinates(self, p1, p2):
        xDiff = p2[0] - p1[0]
        yDiff = p2[1] - p1[1]
        full_angle = np.absolute(np.negative(math.degrees(math.atan2(yDiff, xDiff))) + 90)
        return full_angle


    def getConflictoryTraffic(self):
        """

        :return: (connection, post_intercect_other, [traffic_ids])
        """
        # distances: (dist, interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other, inc, link, outg, index)
        conflictory_traffic = []
        for i in range(len(self.distances)):
            lane_traffic = traci.lane.getLastStepVehicleIDs(self.distances[i][-4])
            connection_traffic = traci.lane.getLastStepVehicleIDs(self.distances[i][-3])
            traffic = []
            for j in range(len(lane_traffic)):
                traffic.append(lane_traffic[j])
            for k in range(len(connection_traffic)):
                traffic.append(connection_traffic[k])
            conflictory_traffic.append((self.distances[i][-3], self.distances[i][5], traffic))
        return conflictory_traffic


    def getDistanceToIntercept(self, traffic):

        traffic_distances = []
        for i in range(len(self.distances)):
            ego_intersection = self.distances[i][-3]
            for j in range(len(traffic)):
                traffic_intersection = traffic[j][0]
                if ego_intersection == traffic_intersection:
                    vehicles = traffic[j][2]
                    distance_to_intercept = []
                    for k in range(len(vehicles)):
                        ego_position = traci.vehicle.getPosition(vehicles[k])
                        ego_angle = traci.vehicle.getAngle(vehicles[k])
                        if ego_angle > 180:
                            ego_angle = 360 - ego_angle
                        coordinate_angle = self.getAngleBetweenCoordinates(ego_position, traffic[j][1])
                        relative_angle = np.absolute(ego_angle - coordinate_angle)
                        if relative_angle < 135:
                            distance_to_intercept.append((vehicles[k], self.getDistance(ego_position, traffic[j][1])))
                    traffic_distances.append((traffic[j][0], traffic[j][1], sorted(distance_to_intercept, key=lambda x: x[1])))
        return traffic_distances



    # def getTimesToIntersection(self, distances):
    #
    #
    #     traffic_times = []
    #     for i in range(len(distances)):
    #         vehicle = distances[i][2]
    #         times_to_intersection = []
    #         for k in range(len(vehicle)):
    #             id, dist = vehicle[k]
    #             speed = np.maximum(traci.vehicle.getSpeed(id), 0.001)
    #             tti = dist / speed
    #             times_to_intersection.append((id, tti, dist))
    #         traffic_times.append((distances[i][0], distances[i][1], sorted(times_to_intersection, key=lambda x: x[1])))
    #
    #     return traffic_times
    #












