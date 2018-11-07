from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import numpy as np
import operator

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

class Framework:



    def removeDuplicates(self, numbers):
        newlist = []
        for number in numbers:
            if number not in newlist:
                newlist.append(number)
        return newlist


    def getTrajectory(self, carID):
        """
        TODO: Further internal link not yet considered
        :param carID: ID of Ego vehicle
        :return: The trajectory of Ego in the form of a list of lane segments
        """
        #print(traci.vehicle.getRoute(carID))
        # Case differentiation
        if len(traci.vehicle.getBestLanes(carID)[0][-1]) > 1:
            if not traci.lane.getLinks(traci.vehicle.getLaneID(carID), extended=True)[0][4]:
                junction_lane = traci.vehicle.getLaneID(carID)
                post_junction_lane = traci.vehicle.getBestLanes(carID)[0][-1][0]
                distance_to_post_junction_lane = traci.lane.getLength(junction_lane) - traci.vehicle.getLanePosition(carID)
                distance_to_subsequent_junction = distance_to_post_junction_lane + traci.lane.getLength(post_junction_lane)
                # if distance_to_post_junction_lane > 100:
                #     return [junction_lane]
                if distance_to_subsequent_junction > 100:
                    return [junction_lane, post_junction_lane]
                elif len(traci.vehicle.getBestLanes(carID)[0][-1]) > 1:
                    # Considering the lane after the second junction
                    subsequent_junction_lane = traci.vehicle.getBestLanes(carID)[0][-1][1]
                    subsequent_junction_connection_list = traci.lane.getLinks(post_junction_lane, extended=True)
                    for j in range(len(subsequent_junction_connection_list)):
                        if subsequent_junction_connection_list[j][0] == subsequent_junction_lane:
                            # Connection between post junction lane and lane after second junction
                            subsequent_junction_connection = (subsequent_junction_connection_list[j][4], subsequent_junction_connection_list[j][-1])
                            distance_to_subsequent_lane = distance_to_subsequent_junction + traci.lane.getLength(post_junction_lane) + subsequent_junction_connection[1]
                            #print('#####')
                            # if distance_to_subsequent_lane > 100:
                            #     return [junction_lane, post_junction_lane, subsequent_junction_connection[0]]
                            # else:
                            return [junction_lane, post_junction_lane, subsequent_junction_connection[0], subsequent_junction_lane]
            else:
                pre_junction_lane = traci.vehicle.getLaneID(carID)
                distance_to_next_junction = traci.lane.getLength(pre_junction_lane) - traci.vehicle.getLanePosition(carID)
                #print(distance_to_next_junction)
                # No junction within range
                if distance_to_next_junction > 100:
                    return [pre_junction_lane]
                # At least one junction within range
                # If length is 1, no other lane will follow in this scene
                # Following junction
                post_junction_lane = traci.vehicle.getBestLanes(carID)[0][-1][1]
                junction_connection_list = traci.lane.getLinks(pre_junction_lane, extended=True)
                for i in range(len(junction_connection_list)):
                    if junction_connection_list[i][0] == post_junction_lane:
                        # List of connection name and length
                        junction_connection = (junction_connection_list[i][4], junction_connection_list[i][-1])
                        distance_to_post_junction_lane = distance_to_next_junction + junction_connection[1]
                        distance_to_subsequent_junction = distance_to_next_junction + junction_connection[1] + traci.lane.getLength(post_junction_lane)

                        # if distance_to_post_junction_lane > 100:
                        #     return [pre_junction_lane, junction_connection[0]]
                        if distance_to_subsequent_junction > 100:
                            return [pre_junction_lane, junction_connection[0], post_junction_lane]
                        elif len(traci.vehicle.getBestLanes(carID)[0][-1]) > 2:
                             # Considering the lane after the second junction
                             subsequent_junction_lane = traci.vehicle.getBestLanes(carID)[0][-1][2]
                             subsequent_junction_connection_list = traci.lane.getLinks(post_junction_lane, extended=True)
                             for j in range(len(subsequent_junction_connection_list)):
                                 if subsequent_junction_connection_list[j][0] == subsequent_junction_lane:
                                     # Connection between post junction lane and lane after second junction
                                     subsequent_junction_connection = (subsequent_junction_connection_list[j][4], subsequent_junction_connection_list[j][-1])
                                     distance_to_subsequent_lane = distance_to_next_junction + junction_connection[1] + traci.lane.getLength(post_junction_lane) + subsequent_junction_connection[1]
                                     subsequent_junction_lane = traci.vehicle.getBestLanes(carID)[0][-1][2]
                                     # if distance_to_subsequent_lane > 100:
                                     #     return [pre_junction_lane, junction_connection[0], post_junction_lane, subsequent_junction_connection[0]]
                                     # else:
                                     return [pre_junction_lane, junction_connection[0], post_junction_lane, subsequent_junction_connection[0], subsequent_junction_lane]
        else:
            if not traci.lane.getLinks(traci.vehicle.getLaneID(carID), extended=True):
                current_lane = traci.vehicle.getLaneID(carID)
                return [current_lane]
            elif not traci.lane.getLinks(traci.vehicle.getLaneID(carID), extended=True)[0][4]:
                junction_lane = traci.vehicle.getLaneID(carID)
                post_junction_lane = traci.vehicle.getBestLanes(carID)[0][-1][0]
                return [junction_lane, post_junction_lane]










    def getRouteInformationOnJunction(self, carID, ego_trajectory):
        ego_trajectory_lanes = traci.vehicle.getBestLanes(carID)[0][-1]
        ego_route = traci.route.getEdges(traci.vehicle.getRouteID(carID))
        lanes_in_scene = traci.lane.getIDList()
        ego_route_lanes = []
        for i in range(len(lanes_in_scene)):
            for j in range(len(ego_route)):
                if traci.lane.getEdgeID(lanes_in_scene[i]) == ego_route[j]:
                    ego_route_lanes.append(lanes_in_scene[i])
        if ego_trajectory[0] not in ego_trajectory_lanes:
            for i in range(len(ego_route_lanes)):
                connections = traci.lane.getLinks(ego_route_lanes[i], extended=True)
                for j in range(len(connections)):
                    if traci.lane.getLinks(ego_route_lanes[i], extended=True)[j][4] == ego_trajectory[0]:
                        pre_junction_lane = ego_route_lanes[i]
                        junction_connection = traci.lane.getLinks(ego_route_lanes[i], extended=True)[j][4]
                        return pre_junction_lane, junction_connection




    def removeInternalIncomingLanes(self, laneIDs):
        real_incoming_lanes = []
        for i in range(len(laneIDs)):
            if ':' not in laneIDs[i]:
                real_incoming_lanes.append(laneIDs[i])
        return real_incoming_lanes






    def getIntersectingTrajectories(self, carID, trajectory):
        """

        :param carID:
        :param trajectory:
        :return: [(incoming_lane, junction_lane, outgoing_lane, junction_affiliation), ]
        """
        ego_trajectory = trajectory
        # End of scene lane
        if not traci.lane.getLinks(traci.vehicle.getLaneID(carID), extended=True):
            return None

        # On junction
        elif not traci.lane.getLinks(traci.vehicle.getLaneID(carID), extended=True)[0][4]:
            pre_junction_lane, junction_connection = self.getRouteInformationOnJunction(carID, ego_trajectory)

            if len(ego_trajectory) >= 4:
                foes_internal_1 = traci.lane.getFoes(junction_connection, '')
                foes_internal_2 = traci.lane.getFoes(ego_trajectory[2], '')
                foes_external_1 = self.removeInternalIncomingLanes(traci.lane.getFoes(pre_junction_lane, ego_trajectory[1]))
                foes_external_2 = self.removeInternalIncomingLanes(traci.lane.getFoes(ego_trajectory[1], ego_trajectory[3]))

                incoming_link_outgoing = []
                utilized_1 = []
                for i in range(len(foes_external_1)):
                    foes_external_links_1 = traci.lane.getLinks(foes_external_1[i], extended=True)
                    for j in range(len(foes_external_links_1)):
                        for k in range(len(foes_internal_1)):
                            if foes_external_links_1[j][4] == foes_internal_1[k] and foes_external_links_1[j][4] not in utilized_1:
                                utilized_1.append(foes_external_links_1[j][4])
                                incoming_link_outgoing.append((foes_external_1[i], foes_external_links_1[j][4], foes_external_links_1[j][0], 1))

                utilized_2 = []
                for i in range(len(foes_external_2)):
                    foes_external_links_2 = traci.lane.getLinks(foes_external_2[i], extended=True)
                    for j in range(len(foes_external_links_2)):
                        for k in range(len(foes_internal_2)):
                            if foes_external_links_2[j][4] == foes_internal_2[k] and foes_external_links_2[j][4] not in utilized_2:
                                utilized_2.append(foes_external_links_2[j][4])
                                incoming_link_outgoing.append((foes_external_2[i], foes_external_links_2[j][4], foes_external_links_2[j][0], 2))

                return incoming_link_outgoing

            elif len(ego_trajectory) > 0:
                foes_internal = traci.lane.getFoes(junction_connection, '')
                foes_external = self.removeInternalIncomingLanes(traci.lane.getFoes(pre_junction_lane, ego_trajectory[0]))

                incoming_link_outgoing = []
                utilized = []
                for i in range(len(foes_external)):
                    foes_external_links = traci.lane.getLinks(foes_external[i], extended=True)
                    for j in range(len(foes_external_links)):
                        for k in range(len(foes_internal)):
                            if foes_external_links[j][4] == foes_internal[k] and foes_external_links[j][4] not in utilized:
                                utilized.append(foes_external_links[j][4])
                                incoming_link_outgoing.append((foes_external[i], foes_external_links[j][4], foes_external_links[j][0], 1))
                return incoming_link_outgoing

        # On lane before junction
        else:
            if len(ego_trajectory) >= 5:
                foes_internal_1 = traci.lane.getFoes(ego_trajectory[1],'')
                foes_internal_2 = traci.lane.getFoes(ego_trajectory[3],'')
                foes_external_1 = self.removeInternalIncomingLanes(traci.lane.getFoes(ego_trajectory[0],ego_trajectory[2]))
                foes_external_2 = self.removeInternalIncomingLanes(traci.lane.getFoes(ego_trajectory[2],ego_trajectory[4]))

                incoming_link_outgoing = []
                utilized_1 = []
                for i in range(len(foes_external_1)):
                    foes_external_links_1 = traci.lane.getLinks(foes_external_1[i], extended=True)
                    for j in range(len(foes_external_links_1)):
                        for k in range(len(foes_internal_1)):
                            if foes_external_links_1[j][4] == foes_internal_1[k] and foes_external_links_1[j][4] not in utilized_1:
                                utilized_1.append(foes_external_links_1[j][4])
                                incoming_link_outgoing.append((foes_external_1[i], foes_external_links_1[j][4], foes_external_links_1[j][0], 1))

                utilized_2 = []
                for i in range(len(foes_external_2)):
                    foes_external_links_2 = traci.lane.getLinks(foes_external_2[i], extended=True)
                    for j in range(len(foes_external_links_2)):
                        for k in range(len(foes_internal_2)):
                            if foes_external_links_2[j][4] == foes_internal_2[k] and foes_external_links_2[j][4] not in utilized_2:
                                utilized_2.append(foes_external_links_2[j][4])
                                incoming_link_outgoing.append((foes_external_2[i], foes_external_links_2[j][4], foes_external_links_2[j][0], 2))
                return incoming_link_outgoing

            elif len(ego_trajectory) >= 3:
                foes_internal = traci.lane.getFoes(ego_trajectory[1],'')
                foes_external = self.removeInternalIncomingLanes(traci.lane.getFoes(ego_trajectory[0],ego_trajectory[2]))

                incoming_link_outgoing = []
                utilized = []
                for i in range(len(foes_external)):
                    foes_external_links = traci.lane.getLinks(foes_external[i], extended=True)
                    for j in range(len(foes_external_links)):
                        for k in range(len(foes_internal)):
                            if foes_external_links[j][4] == foes_internal[k] and foes_external_links[j][4] not in utilized:
                                utilized.append(foes_external_links[j][4])
                                incoming_link_outgoing.append((foes_external[i], foes_external_links[j][4], foes_external_links[j][0], 1))
                return incoming_link_outgoing









    def getEuclideanDistance(self, shape1, shape2):
        return np.sqrt(np.sum(np.square(np.subtract(shape1, shape2))))


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





    def getNearestNeighbor(self, function1, function2):


        for i in range(len(function1)):
            for j in range(len(function2)):
                if self.getEuclideanDistance(function1[i], function2[j]) < 1:
                    return function1[i], function2[j]
        return 0, 0



    def getPrePostNeighbor(self, function1, function2):
        """
        TODO: If index out of range, a supplementary coordinate has to be generated
        :param function1: Ego trajectory funciton
        :param function2: Traffic vehicle trajectory function
        :return: ego_poc, real_ego_poc, trc_poc, trc_pre_poc, trc_post_poc
        """
        for i in range(len(function1)):
            for j in range(len(function2)):
                if self.getEuclideanDistance(function1[i], function2[j]) < 1:
                    return function1[i+1], function1[i-1], function2[j], function2[j-1], function2[j+1]
        return 0, 0





    def getTensorFramework(self, carID, intersecting_trajectories, trajectory):
        """

        :param carID:
        :param intersections:
        :return: List of tuples: (distance_to_connection, intercept_coordinate, intersecting_trajectory)
        """

        if not traci.lane.getLinks(traci.vehicle.getLaneID(carID), extended=True):
            return None

        # On junction
        elif not traci.lane.getLinks(traci.vehicle.getLaneID(carID), extended=True)[0][4]:
            pre_junction_lane, junction_connection = self.getRouteInformationOnJunction(carID, trajectory)
            if len(trajectory) >= 4:
                connection_1 = junction_connection
                ego_trajectory_function_1 = self.getTrajectoryFunction(connection_1)
                trajectory_intercepts = []
                for i in range(len(intersecting_trajectories)):
                    if intersecting_trajectories[i][-1] == 1:
                        interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other = self.getPrePostNeighbor(ego_trajectory_function_1,
                                                                                                                                 self.getTrajectoryFunction(intersecting_trajectories[i][1]))
                        if interc_ego is not 0 and interc_other is not 0:
                            inc, link, outg, index = intersecting_trajectories[i]
                            trajectory_intercepts.append((interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other, inc, link, outg, index))

                connection_2 = trajectory[2]
                #print(connection_2)
                ego_trajectory_function_2 = self.getTrajectoryFunction(connection_2)
                #print(intersecting_trajectories)
                #trajectory_intercepts_2 = []
                for i in range(len(intersecting_trajectories)):
                    if intersecting_trajectories[i][-1] == 2:
                        interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other = self.getPrePostNeighbor(ego_trajectory_function_2,
                                                                                                                                 self.getTrajectoryFunction(intersecting_trajectories[i][1]))
                        if interc_ego is not 0 and interc_other is not 0:
                            inc, link, outg, index = intersecting_trajectories[i]
                            trajectory_intercepts.append((interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other, inc, link, outg, index))
                #print(trajectory_intercepts)

                return trajectory_intercepts

            elif len(trajectory) >= 2:
                connection = trajectory[0]
                ego_trajectory_function = self.getTrajectoryFunction(connection)
                trajectory_intercepts = []
                for i in range(len(intersecting_trajectories)):
                    if intersecting_trajectories[i][-1] == 1:
                        interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other = self.getPrePostNeighbor(ego_trajectory_function,
                                                                                                                                 self.getTrajectoryFunction(intersecting_trajectories[i][1]))
                        if interc_ego is not 0 and interc_other is not 0:
                            inc, link, outg, index = intersecting_trajectories[i]
                            trajectory_intercepts.append((interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other, inc, link, outg, index))
                #print(trajectory_intercepts)
                return trajectory_intercepts



        else:

            if len(trajectory) >= 5:
                connection_1 = trajectory[1]
                ego_trajectory_function_1 = self.getTrajectoryFunction(connection_1)
                trajectory_intercepts = []
                for i in range(len(intersecting_trajectories)):
                    if intersecting_trajectories[i][-1] == 1:
                        interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other = self.getPrePostNeighbor(ego_trajectory_function_1,
                                                                                                                                 self.getTrajectoryFunction(intersecting_trajectories[i][1]))
                        if interc_ego is not 0 and interc_other is not 0:
                            inc, link, outg, index = intersecting_trajectories[i]
                            trajectory_intercepts.append((interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other, inc, link, outg, index))

                connection_2 = trajectory[3]
                ego_trajectory_function_2 = self.getTrajectoryFunction(connection_2)
                for i in range(len(intersecting_trajectories)):
                    if intersecting_trajectories[i][-1] == 2:
                        interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other = self.getPrePostNeighbor(ego_trajectory_function_2,
                                                                                                                                 self.getTrajectoryFunction(intersecting_trajectories[i][1]))
                        if interc_ego is not 0 and interc_other is not 0:
                            inc, link, outg, index = intersecting_trajectories[i]
                            trajectory_intercepts.append((interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other, inc, link, outg, index))
                return trajectory_intercepts

            elif len(trajectory) >= 3:
                connection = trajectory[1]
                ego_trajectory_function = self.getTrajectoryFunction(connection)
                trajectory_intercepts = []
                for i in range(len(intersecting_trajectories)):
                    if intersecting_trajectories[i][-1] == 1:
                        interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other = self.getPrePostNeighbor(ego_trajectory_function, self.getTrajectoryFunction(intersecting_trajectories[i][1]))
                        if interc_ego is not 0 and interc_other is not 0:
                            inc, link, outg, index = intersecting_trajectories[i]
                            trajectory_intercepts.append((interc_ego, real_interc_ego, interc_other, pre_interc_other, post_interc_other, inc, link, outg, index))
                return trajectory_intercepts


















