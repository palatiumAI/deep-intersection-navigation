from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import numpy as np
import operator
from math import sin, radians, log10

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

class Safety:

    def __init__(self, carID, framework, distances, distances_tfc):
        self.carID = carID
        # framework: (interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other, inc, link, outg, index)
        self.framework = framework
        # distances: (dist, interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other, inc, link, outg, index)
        self.distances = distances
        # distances_tfc: (link, interc_other, [(trc_id1, dist), ...])
        self.distances_tfc = distances_tfc


    def avoidDivisionByZero(self, number):
        """
        Avoids division by zero
        """
        if number == 0:
            return 0.01
        else:
            return number

    def getDistance(self, position1, position2):
        x1, y1 = position1
        x2, y2 = position2
        dist = traci.simulation.getDistance2D(x1, y1, x2, y2, isDriving=False)
        return dist

    def getEuclideanDistance(self, shape1, shape2):
        return np.sqrt(np.sum(np.square(np.subtract(shape1, shape2))))


    def getTrajectoryDistance(self, intersecting_trajectory):
        """

        :param intersecting_trajectory: Junction connection
        :return: List of coordinates describing the trajectory
        """

        further_link = []
        add = True
        while add:
            if not further_link:
                if traci.lane.getLinks(intersecting_trajectory, extended=True)[0][4]:
                    further_link.append(traci.lane.getLinks(intersecting_trajectory, extended=True)[0][4])
                else:
                    add = False
            else:
                if traci.lane.getLinks(further_link[-1], extended=True)[0][4]:
                    further_link.append(traci.lane.getLinks(further_link[-1], extended=True)[0][4])
                else:
                    add = False

        if not further_link:
            shape = traci.lane.getShape(intersecting_trajectory)
            distance = 0
            for i in range(len(shape)):
                if i+1 < len(shape):
                    euklid_dist = self.getEuclideanDistance(shape[i+1], shape[i])
                    distance = distance + euklid_dist
            return distance

        else:
            shape = traci.lane.getShape(intersecting_trajectory)
            distance = 0
            for i in range(len(shape)):
                if i + 1 < len(shape):
                    euklid_dist = self.getEuclideanDistance(shape[i + 1], shape[i])
                    distance = distance + euklid_dist

            for i in range(len(further_link)):
                shape = traci.lane.getShape(further_link[i])
                for i in range(len(shape)):
                    if i + 1 < len(shape):
                        euklid_dist = self.getEuclideanDistance(shape[i + 1], shape[i])
                        distance = distance + euklid_dist
            return distance



    def getTrajectoryDistanceToIntercept(self, intersecting_trajectory, intercept, reverse=False):
        """
        TODO: euklid_dist_to_intercept may be too small -> maybe heuristic approach
        :param intersecting_trajectory: Junction connection
        :return: List of coordinates describing the trajectory
        """

        further_link = []
        add = True
        while add:
            if not further_link:
                if traci.lane.getLinks(intersecting_trajectory, extended=True)[0][4]:
                    further_link.append(traci.lane.getLinks(intersecting_trajectory, extended=True)[0][4])
                else:
                    add = False
            else:
                if traci.lane.getLinks(further_link[-1], extended=True)[0][4]:
                    further_link.append(traci.lane.getLinks(further_link[-1], extended=True)[0][4])
                else:
                    add = False

        if not further_link:
            shape = self.getTrajectoryFunction(intersecting_trajectory)
            #print(shape)
            if reverse:
                 shape.reverse()
            #print(shape)
            distance = 0
            for i in range(len(shape)):
                if i+1 < len(shape):
                    euklid_dist_to_intercept = self.getEuclideanDistance(intercept, shape[i])
                    #print('#',euklid_dist_to_intercept)

                    if euklid_dist_to_intercept < 2.7:
                        distance = distance + euklid_dist_to_intercept
                        return distance
                    else:
                        euklid_dist = self.getEuclideanDistance(shape[i + 1], shape[i])
                        distance = distance + euklid_dist

        else:
            shape = self.getTrajectoryFunction(intersecting_trajectory)
            if reverse:
                shape = shape.reverse()
            distance = 0
            for i in range(len(shape)):
                if i + 1 < len(shape):
                    euklid_dist_to_intercept = self.getEuclideanDistance(intercept, shape[i])
                    #print('##',euklid_dist_to_intercept)

                    if euklid_dist_to_intercept < 2.7:
                        distance = distance + euklid_dist_to_intercept
                        return distance
                    else:
                        euklid_dist = self.getEuclideanDistance(shape[i + 1], shape[i])
                        distance = distance + euklid_dist

            for i in range(len(further_link)):
                shape = self.getTrajectoryFunction(further_link[i])
                if reverse:
                    shape = shape.reverse()
                for i in range(len(shape)):
                    if i + 1 < len(shape):
                        euklid_dist_to_intercept = self.getEuclideanDistance(intercept, shape[i])
                        if euklid_dist_to_intercept < 2.7:
                            distance = distance + euklid_dist_to_intercept
                            return distance
                    else:
                        euklid_dist = self.getEuclideanDistance(shape[i + 1], shape[i])
                        distance = distance + euklid_dist


    def getEquidistantPoints(self, p1, p2, parts):
        points = list(zip(np.linspace(p1[0], p2[0], parts+1, endpoint=False), np.linspace(p1[1], p2[1], parts+1, endpoint=False)))
        return points

    def getTrajectoryFunction(self, intersecting_trajectory):
        """

        :param carID:
        :param intersecting_trajectory: Junction connection
        :return: List of coordinates describing the trajectory
        """

        further_link = []
        add = True
        while add:
            if not further_link:
                if traci.lane.getLinks(intersecting_trajectory, extended=True)[0][4]:
                    further_link.append(traci.lane.getLinks(intersecting_trajectory, extended=True)[0][4])
                else:
                    add = False
            else:
                if traci.lane.getLinks(further_link[-1], extended=True)[0][4]:
                    further_link.append(traci.lane.getLinks(further_link[-1], extended=True)[0][4])
                else:
                    add = False

        if not further_link:
            sparse_shape = traci.lane.getShape(intersecting_trajectory)
            interpolated_shape = []
            for i in range(len(sparse_shape)):
                if i+1 < len(sparse_shape):
                    euklid_dist = self.getEuclideanDistance(sparse_shape[i+1], sparse_shape[i])
                    if euklid_dist > 2:
                        num_add_points_f = round(euklid_dist) / 2
                        num_add_points = int(num_add_points_f)
                        points = self.getEquidistantPoints(sparse_shape[i], sparse_shape[i+1], num_add_points)
                        for j in range(len(points)):
                            interpolated_shape.append(points[j])
                    else:
                        interpolated_shape.append(sparse_shape[i])
            if interpolated_shape[-1] != sparse_shape[-1]:
                interpolated_shape.append(sparse_shape[-1])
            return interpolated_shape

        else:
            interpolated_shape = []

            sparse_shape = traci.lane.getShape(intersecting_trajectory)
            for i in range(len(sparse_shape)):
                if i + 1 < len(sparse_shape):
                    euklid_dist = self.getEuclideanDistance(sparse_shape[i + 1], sparse_shape[i])
                    if euklid_dist > 2:
                        num_add_points_f = round(euklid_dist) / 2
                        num_add_points = int(num_add_points_f)
                        points = self.getEquidistantPoints(sparse_shape[i], sparse_shape[i + 1], num_add_points)
                        for j in range(len(points)):
                            interpolated_shape.append(points[j])
                    else:
                        interpolated_shape.append(sparse_shape[i])
            if interpolated_shape[-1] != sparse_shape[-1]:
                interpolated_shape.append(sparse_shape[-1])

            for i in range(len(further_link)):
                sparse_shape = traci.lane.getShape(further_link[i])
                for i in range(len(sparse_shape)):
                    if i + 1 < len(sparse_shape):
                        euklid_dist = self.getEuclideanDistance(sparse_shape[i + 1], sparse_shape[i])
                        if euklid_dist > 2:
                            num_add_points_f = round(euklid_dist) / 2
                            num_add_points = int(num_add_points_f)
                            points = self.getEquidistantPoints(sparse_shape[i], sparse_shape[i + 1], num_add_points)
                            for j in range(len(points)):
                                interpolated_shape.append(points[j])
                        else:
                            interpolated_shape.append(sparse_shape[i])
                if interpolated_shape[-1] != sparse_shape[-1]:
                    interpolated_shape.append(sparse_shape[-1])
            return interpolated_shape

    #def catchInf(self, number):



    def getTrafficSafetyMeasures(self):
        ego_width = traci.vehicle.getWidth(self.carID)
        tfc_safety_meas = []
        for i in range(len(self.distances_tfc)):
            dist_of_link = self.getTrajectoryDistance(self.distances_tfc[i][0])
            dist_to_intercept = self.getTrajectoryDistanceToIntercept(self.distances_tfc[i][0], self.distances[i][1])
            tfc_tbi_tai = []
            for j in range(len(self.distances_tfc[i][2])):
                lane_dist_to_conneciton = traci.lane.getLength(self.distances[i][-4]) - traci.vehicle.getLanePosition(self.distances_tfc[i][2][j][0])
                trc_id = self.distances_tfc[i][2][j][0]
                trc_length = traci.vehicle.getLength(trc_id)
                connection_dist_to_pbi = dist_to_intercept - ego_width / 2
                connection_dist_to_pai = dist_to_intercept + ego_width / 2 + trc_length
                if traci.vehicle.getLaneID(self.distances_tfc[i][2][j][0]) == self.distances[i][-4]:
                    dist_to_pbi = np.maximum(lane_dist_to_conneciton + connection_dist_to_pbi, 0)
                    tbi = dist_to_pbi / self.avoidDivisionByZero(traci.vehicle.getSpeed(trc_id))
                    dist_to_pai = np.maximum(lane_dist_to_conneciton + connection_dist_to_pai, 0)
                    tai = dist_to_pai / self.avoidDivisionByZero(traci.vehicle.getSpeed(trc_id))
                    tfc_tbi_tai.append((trc_id, tbi, tai))
                else:
                    tfc_dist_to_eol = self.getTrajectoryDistanceToIntercept(self.distances_tfc[i][0], traci.vehicle.getPosition(self.distances_tfc[i][2][j][0]), True)
                    traveled_dist_on_link = dist_of_link - tfc_dist_to_eol
                    dist_to_pbi = np.maximum(connection_dist_to_pbi - traveled_dist_on_link, 0)
                    tbi = dist_to_pbi / self.avoidDivisionByZero(traci.vehicle.getSpeed(trc_id))
                    dist_to_pai = np.maximum(connection_dist_to_pai - traveled_dist_on_link, 0)
                    tai = dist_to_pai / self.avoidDivisionByZero(traci.vehicle.getSpeed(trc_id))
                    tfc_tbi_tai.append((trc_id, tbi, tai))
            tfc_safety_meas.append((self.distances_tfc[i][0], self.distances[i][1], tfc_tbi_tai))
        return tfc_safety_meas






    def getEgoSafetyMeasures(self):

        ego_speed = np.maximum(traci.vehicle.getSpeed(self.carID), 0.01)
        #ego_speed = traci.vehicle.getSpeed(self.carID)

        times_to_intersection = []
        for i in range(len(self.distances)):
            dist, interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other, inc, link, outg, index = self.distances[i]
            tti = dist / ego_speed
            times_to_intersection.append((link, interc_ego, tti))
        return times_to_intersection

    def getXEgoSafetyMeasures(self):
        ego_speed = np.maximum(traci.vehicle.getSpeed(self.carID), 0.01)

        #ego_speed = np.maximum(traci.vehicle.getSpeed(self.carID), 0.3)
        #ego_speed = traci.vehicle.getSpeed(self.carID)

        times_to_intersection = []
        for i in range(len(self.distances)):
            dist, interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other, inc, link, outg, index = self.distances[i]
            tti = dist / 3.5
            times_to_intersection.append((link, interc_ego, tti))
        return times_to_intersection





    def getPriotizedTraffic(self, ego_safety, trc_safety):

        priority_measures_per_link = []
        for i in range(len(ego_safety)):
            ego_tti = ego_safety[i][2]
            priority_measures = []
            for j in range(len(trc_safety[i][2])):
                trc_id, trc_tti, tai = trc_safety[i][2][j]
                priority_measures.append((trc_id, np.absolute(trc_tti - ego_tti), trc_tti, tai))
            priority_measures_per_link.append((ego_safety[i][0], sorted(priority_measures, key=lambda x: x[1])))
        return priority_measures_per_link

    def getNearestTraffic(self, ego_safety, trc_safety):

        priority_measures_per_link = []
        for i in range(len(ego_safety)):
            ego_tti = ego_safety[i][2]
            priority_measures = []
            for j in range(len(trc_safety[i][2])):
                trc_id, trc_tti, tai = trc_safety[i][2][j]
                priority_measures.append((trc_id, np.absolute(trc_tti - ego_tti), trc_tti, tai))
            priority_measures_per_link.append((ego_safety[i][0], sorted(priority_measures, key=lambda x: x[2])))
        return priority_measures_per_link





    def getRelevantTraffic(self, prio, safety_measure):

        current_speed = traci.vehicle.getSpeed(self.carID)
        allowed_speed = traci.lane.getMaxSpeed(traci.vehicle.getLaneID(self.carID))

        quot = allowed_speed / self.avoidDivisionByZero(current_speed)

        gap_multiplier = np.minimum(log10(quot) + 1, 1.5)
        time_gap = safety_measure * 2.5 * gap_multiplier

        relevant_traffic = []
        for i in range(len(prio)):
            if prio[i][1]:
                relevant = [(prio[i][1][0][0], prio[i][1][0][2], prio[i][1][0][3])]
                if len(prio[i][1]) > 1:
                    for j in range(len(prio[i][1])):
                        if j+1 < len(prio[i][1]):
                            # Check if next vehicle drives behind
                            if relevant[-1][1] - prio[i][1][j+1][2] < 0:
                                if relevant[-1][1] - prio[i][1][j + 1][2] > -time_gap:
                                    relevant.append((prio[i][1][j+1][0], prio[i][1][j+1][2], prio[i][1][j+1][3]))
                            else:
                                if relevant[0][1] - prio[i][1][j + 1][2] < time_gap:
                                    relevant.insert(0, (prio[i][1][j+1][0], prio[i][1][j+1][2], prio[i][1][j+1][3]))
                propagated_relevant = [(prio[i][1][0][0], relevant[0][1], relevant[-1][2])]
                relevant_traffic.append((prio[i][0], propagated_relevant))
            else:
                relevant_traffic.append((prio[i][0], [(None, -1, -1)]))
        return relevant_traffic


    def getNearestRelevantTraffic(self, prio, safety_measure):

        current_speed = traci.vehicle.getSpeed(self.carID)
        allowed_speed = traci.lane.getMaxSpeed(traci.vehicle.getLaneID(self.carID))

        quot = allowed_speed / self.avoidDivisionByZero(current_speed)

        gap_multiplier = np.minimum(log10(quot) + 1, 1.5)
        time_gap = safety_measure * 2.5 * gap_multiplier

        relevant_traffic = []
        for i in range(len(prio)):
            if prio[i][1]:
                relevant = [(prio[i][1][0][0], prio[i][1][0][2], prio[i][1][0][3])]
                if len(prio[i][1]) > 1:
                    for j in range(len(prio[i][1])):
                        if j+1 < len(prio[i][1]):
                            # Check if next vehicle drives behind
                            if relevant[-1][1] - prio[i][1][j+1][2] < 0:
                                if relevant[-1][1] - prio[i][1][j + 1][2] > -time_gap:
                                    relevant.append((prio[i][1][j+1][0], prio[i][1][j+1][2], prio[i][1][j+1][3]))
                            else:
                                if relevant[0][1] - prio[i][1][j + 1][2] < time_gap:
                                    relevant.insert(0, (prio[i][1][j+1][0], prio[i][1][j+1][2], prio[i][1][j+1][3]))
                propagated_relevant = [(prio[i][1][0][0], relevant[0][1], relevant[-1][2])]
                relevant_traffic.append((prio[i][0], propagated_relevant))
            else:
                relevant_traffic.append((prio[i][0], [(None, -1, -1)]))
        return relevant_traffic










