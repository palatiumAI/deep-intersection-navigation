


import numpy as np
#from env_tensor import Environment
import traci
import sumolib

class Reward:

    #net = sumolib.net.readNet('huge_crossing/huge_crossing.net.xml')
    #net = sumolib.net.readNet('one_lane/one_lane.net.xml')

    # def __init__(self, carID, tensor):
    #     self.carID = carID
    #     self.tensor = tensor

    def __init__(self, carID, rel, tensor, prio):
        self.carID = carID
        self.conflictories = rel
        self.tensor = tensor
        self.prioritized_traffic = prio


####################################################################################################
    # def optimum_speed_deviation(self):
    #     allowed_speed = traci.lane.getMaxSpeed(traci.vehicle.getLaneID(self.carID))
    #     current_speed = traci.vehicle.getSpeed(self.carID)
    #     deviation = allowed_speed - current_speed
    #     if 7 > deviation >= 0:
    #         return 1 / np.maximum(deviation, 1)
    #     elif -2.5 < deviation < 0:
    #         return 1 / np.maximum(-deviation, 1)
    #     elif deviation < -2.5:
    #         return 0.1 * deviation * 3
    #
    #     else:
    #         ### ### return -0.1 * (deviation / 2)
    #         return -0.1 * (deviation / 3)
    def optimum_speed_deviation(self):
        allowed_speed = traci.lane.getMaxSpeed(traci.vehicle.getLaneID(self.carID))
        current_speed = traci.vehicle.getSpeed(self.carID)
        deviation = current_speed / allowed_speed

        if 1.05 >= deviation >= 0.95:
            return 1
        elif deviation > 1.05:
            return -(deviation - 1) / 1.05

        elif 0.95 > deviation >= 0.5:
            return (2 * deviation - 1) * 1.05
        else:
            return (deviation / 2.5) - 0.2
            #return (deviation / 5) - 0.1

    ####################################################################################################

    # def collision(self):
    #     pain = 10000
    #     num_vec = traci.vehicle.getIDList()
    #     #print(num_vec)
    #     euklid_dist_list = []
    #     for i in range(len(num_vec)):
    #         if num_vec[i] != 'ego':
    #             euklid_dist = np.sqrt(np.sum(np.square(np.subtract(traci.vehicle.getPosition('ego'), traci.vehicle.getPosition(num_vec[i])))))
    #             euklid_dist_list.append(euklid_dist)
    #
    #     for j in range(len(euklid_dist_list)):
    #         if euklid_dist_list[j] < 1:
    #             return np.negative(pain), True
    #     return 0, False





####################################################################################################
    def collision(self):
        #print('###', traci.simulation.getCollidingVehiclesIDList())
        #if 'ego' in traci.simulation.getCollidingVehiclesIDList():
        if len(traci.simulation.getStartingTeleportIDList()) > 0:
            print('## Collision ##')
            return -50, True
            #-1000000, True
        else:
            return 0, False

    def collision_test(self):
        #print('###', traci.simulation.getCollidingVehiclesIDList())
        #if 'ego' in traci.simulation.getCollidingVehiclesIDList():
        if len(traci.simulation.getStartingTeleportIDList()) > 0:
            print('## Collision ##')
            return -50, True
            #-1000000, True
        else:
            return 0, False
####################################################################################################

    # def emergency_brake(self):
    #     #print(self.conflictories)
    #     if self.conflictories:
    #         pain = 0
    #         for i in range(len(self.conflictories)):
    #             #print(self.conflictories[i])
    #             if traci.vehicle.getPosition('ego') is not None:
    #                 if self.conflictories[i][2]:
    #                     euklid_dist = np.sqrt(np.sum(np.square(np.subtract(traci.vehicle.getPosition('ego'), traci.vehicle.getPosition(self.conflictories[i][2][0][0])))))
    #                     allowed_speed = traci.lane.getMaxSpeed(traci.vehicle.getLaneID(self.conflictories[i][2][0][0]))
    #                     current_speed = traci.vehicle.getSpeed(self.conflictories[i][2][0][0])
    #                     deviation = 1 / np.maximum((current_speed / allowed_speed), 000.1)
    #                     signals = traci.vehicle.getSignals(self.conflictories[i][2][0][0])
    #
    #                     if signals == 8 and euklid_dist < 30:
    #                         pain = pain + np.negative(deviation * 500)
    #             #print('###',pain)
    #             return pain
    #     else:
    #         return 0


    # def emergency_gap(self):
    #     critical_space = self.tensor[0:200]
    #
    #     allowed_speed = traci.lane.getMaxSpeed(traci.vehicle.getLaneID(self.carID))
    #     current_speed = traci.vehicle.getSpeed(self.carID)
    #     deviation = allowed_speed - current_speed
    #
    #     for i in range(len(critical_space)):
    #         if critical_space[i, 0] < critical_space[i, 2] < critical_space[i, 1]:
    #
    #             if self.prioritized_traffic:
    #                 for j in range(len(self.prioritized_traffic)):
    #                     if self.prioritized_traffic[j][1]:
    #                         if critical_space[i][2] + self.prioritized_traffic[j][1][0][2] < 1:
    #                             print('# Collision #')
    #                             return -50000, True
    #                         elif critical_space[i][2] + self.prioritized_traffic[j][1][0][2] < 10:
    #                             wary = 0
    #                             # if deviation > 0:
    #                             #     wary = np.absolute(np.power(deviation, 4))
    #                             #     print(wary)
    #
    #                             #return -15000 + wary, False
    #                             return -15000, False
    #
    #                         else:
    #                         #     wary = 0
    #                         #     if deviation > 0:
    #                         #         wary = np.absolute(np.power(deviation, 4))
    #                         #         print(wary)
    #                             #return -7500 + wary, False
    #
    #                             return -7500, False
    #
    #     return 0, False

    # def emergency_gap(self):
    #     critical_space = self.tensor[0:200]
    #
    #
    #     for i in range(len(critical_space)):
    #         if critical_space[i, 0] < critical_space[i, 2] < critical_space[i, 1]:
    #
    #             if self.prioritized_traffic:
    #                 for j in range(len(self.prioritized_traffic)):
    #                     if self.prioritized_traffic[j][1]:
    #                         if critical_space[i][2] < 1:
    #                             print('# Collision #')
    #                             return -50000, True








####################################################################################################
    def emergency_gap(self):
        critical_space = self.tensor[0:200]
        allowed_speed = traci.lane.getMaxSpeed(traci.vehicle.getLaneID(self.carID))
        current_speed = traci.vehicle.getSpeed(self.carID)
        deviation = allowed_speed - current_speed

        for i in range(len(critical_space)):
            if critical_space[i, 0] < critical_space[i, 2] < critical_space[i, 1]:
                #print('     collision course')

                if self.prioritized_traffic:
                    for j in range(len(self.prioritized_traffic)):
                        if self.prioritized_traffic[j][1]:
                            if critical_space[i][2] < 1:
                                print('# Collision #')
                                return -50, True #-1000000, True
                            # else:
                            #     if deviation > 0:
                            #         return np.power(deviation, 3) + np.power(deviation, 3), False
                            #     else:
                            #         return 0, False

        return 0, False
####################################################################################################
    def emergency_gap_test(self):
        critical_space = self.tensor[0:200]
        allowed_speed = traci.lane.getMaxSpeed(traci.vehicle.getLaneID(self.carID))
        current_speed = traci.vehicle.getSpeed(self.carID)
        deviation = allowed_speed - current_speed

        for i in range(len(critical_space)):
            if critical_space[i, 0] < critical_space[i, 2] < critical_space[i, 1]:
                #print('     collision course')
                if self.prioritized_traffic:
                    for j in range(len(self.prioritized_traffic)):
                        if self.prioritized_traffic[j][1]:
                            if critical_space[i][2] < 1:
                                print('# Collision #')
                                return -50, True  # -1000000, True
                # if self.prioritized_traffic:
                #     for j in range(len(self.prioritized_traffic)):
                #         if self.prioritized_traffic[j][1]:
                #             if critical_space[i][2] < 1:
                #                 print('# Collision #')
                #                 return -100, True #-1000000, True
                            # else:
                            #     if deviation > 0:
                            #         return np.power(deviation, 3) + np.power(deviation, 3), False
                            #     else:
                            #         return 0, False

        return 0, False









    # def emergency_gap_test(self):
    #     critical_space = self.tensor[0:200]
    #
    #     allowed_speed = traci.lane.getMaxSpeed(traci.vehicle.getLaneID(self.carID))
    #     current_speed = traci.vehicle.getSpeed(self.carID)
    #     deviation = allowed_speed - current_speed
    #
    #     for i in range(len(critical_space)):
    #         if critical_space[i, 0] < critical_space[i, 2] < critical_space[i, 1]:
    #
    #             if self.prioritized_traffic:
    #                 for j in range(len(self.prioritized_traffic)):
    #                     if self.prioritized_traffic[j][1]:
    #                         #print(critical_space[i][2], self.prioritized_traffic[j][1][0][2])
    #                         if critical_space[i][2] + self.prioritized_traffic[j][1][0][2] < 1:
    #                             print('### Collision ###')
    #                             return -30000, True
    #                         elif critical_space[i][2] + self.prioritized_traffic[j][1][0][2] < 10:
    #                             wary = 0
    #                             if deviation > 0:
    #                                 wary = np.absolute(np.power(deviation, 4.5))
    #
    #                                 print(' # # #',wary)
    #
    #                             print('## Critical Collision Course ##')
    #
    #                             return -15000 + wary, False
    #                         else:
    #                             wary = 0
    #                             if deviation > 0:
    #                                 wary = np.absolute(np.power(deviation, 4.5))
    #
    #                                 print(' # # #',wary)
    #
    #                             print('# Collision course #')
    #
    #                             return -7500 + wary, False
    #
    #     return 0, False



    # def wary_before_intersection(self):
    #     critical_space = self.tensor[0:200]
    #     allowed_speed = traci.lane.getMaxSpeed(traci.vehicle.getLaneID(self.carID))
    #     current_speed = traci.vehicle.getSpeed(self.carID)
    #     deviation = allowed_speed - current_speed
    #
    #     if critical_space[0, 2] == 0:
    #         for i in range(len(critical_space)):
    #             j = np.negative(i + 1)
    #             if critical_space[j, 2] != 0:
    #                 if deviation > 0:
    #                     return np.absolute(np.power(deviation, 2))
    #                 else:
    #                     return 0
    #             else:
    #                 return 0
    #     else:
    #         return 0



    # def overtime(self):
    #     if traci.simulation.getCurrentTime() < 100000:
    #         return 0, False
    #     else:
    #         return 0, False
    #         #return -2000000, True

























