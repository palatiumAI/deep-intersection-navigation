from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

from Deep_Intersection_Navigation.representation.assembler import Assembler
import sumolib
import traci
import numpy as np
#from helper import Helper
from reward import Reward
from action import Action


from tensorforce.environments import Environment



class Engine(Environment):

    def __init__(self, carID):
        self.carID = carID
        self.output = None
        self.divisor = 1 / 20


    def __str__(self):
        return 'Train Engine'

    def setDivisor(self, input):
        self.divisor = input



    def reset(self):
        self.generate_routefile()

        traci.load(["-c", "../mapfiles/one_lane/one_lane.sumocfg", "--collision.check-junctions", "1", '--no-step-log', 'true', '-W', 'true',"--start", '--time-to-teleport', '60'])




        run = Assembler(self.carID)
        terminate = False
        while terminate == False:

            num_vec = traci.vehicle.getIDList()
            for i in range(len(num_vec)):
                if num_vec[i] != 'ego':
                    traci.vehicle.setLaneChangeMode(num_vec[i], 512)
            self.output = run.getTensor()
            if self.output is not None:
                terminate = True
            traci.simulationStep()

        return self.getObservation()


    def close(self):
        traci.close()


    def execute(self, action):
        #print('execute')
        run = Assembler(self.carID)

        self.output = run.getTensor()
        if self.output is None:
            term = True
            #print('is none')
            return self.getObservation(), term, 0 #20000

        rew = Reward('ego', run.getTraffic(), self.output, run.getPrioritizedTraffic())

        coll, term = rew.collision()
        if term is True:
            cost = coll
            return self.getObservation(), term, cost

        # over, term = rew.overtime()
        # if term is True:
        #     cost = over
        #     return self.getObservation(), term, cost

        traci.vehicle.setSpeedMode('ego', 0)
        num_vec = traci.vehicle.getIDList()
        for i in range(len(num_vec)):
            if num_vec[i] != 'ego':
                traci.vehicle.setLaneChangeMode(num_vec[i], 512)
        carID = 'ego'
        act = Action(carID)
        #print(action)
        if action == 0:
            act.decelerate()
            #print('dec')
        elif action == 1:
            act.accelerate()
            #print('acc')
        #elif action == 2:
         #   act.emergency()
        else:
            act.remain()
            #print('rem')

        gap, term = rew.emergency_gap()
        #wary = rew.wary_before_intersection()

        #brake = rew.emergency_brake()
        #print(self.output[0:20])
        #print(gap)
        cost = rew.optimum_speed_deviation() + gap #+ over
        traci.simulationStep()
        #print(self.getObservation())
        #print(term)

        return self.getObservation(), term, cost


    @property
    def states(self):
        return dict(shape=(600, ), type='float')


    @property
    def actions(self):
        return dict(num_actions=3, type='int')


    def getObservation(self):
        if self.output is not None:
            #print(self.output)

            tensor = np.reshape(self.output[0:200], 600)
        else:

            tensor = None
        return tensor



    def generate_routefile(self):
        #np.random.seed(42)  # make tests reproducible
        N = 200  # number of time steps
        # demand per second from different directions
        pN1 = self.divisor #1. / 15
        pN2 = 1. / 3
        pS1 = self.divisor #1. / 15
        pS2 = 2. / 3
        with open("../mapfiles/one_lane/one_lane.rou.xml", "w") as routes:
            print("""<routes>
            <vType accel="1.0" decel="5.0" id="Car" length="4.0" maxSpeed="20.0" sigma="0.0" departSpeed="13.89"/>
    
            <route id="n2s" edges="gneE2 -gneE3" />
            <route id="s2n" edges="gneE3 -gneE2" />
            <route id="w2e" edges="gneE0 -gneE1" />
    
            """, file=routes)
            lastVeh = 0
            vehNr = 0
            for i in range(N):
                if np.random.uniform(0, 1) < pN1:
                    print('    <vehicle id="right_%i" type="Car" route="n2s" depart="%i" />' % (vehNr, i), file=routes)
                    vehNr += 1
                    lastVeh = i
                # if np.random.uniform(0, 1) < pN2:
                #     print('    <vehicle id="right_%i" type="Car" route="n2s" depart="%i" />' % (vehNr, i), file=routes)
                #     vehNr += 1
                #     lastVeh = i
                if i == 15:
                    print('    <vehicle id="ego" type="Car" route="w2e" depart="%i" departSpeed="13.89"/>' % i, file=routes)
                    vehNr += 1
                    lastVeh = i
                if np.random.uniform(0, 1) < pS1:
                    print('    <vehicle id="left_%i" type="Car" route="s2n" depart="%i" />' % (
                        vehNr, i), file=routes)
                    vehNr += 1
                    lastVeh = i
                # if np.random.uniform(0, 1) < pS2:
                #     print('    <vehicle id="left_%i" type="Car" route="s2n" depart="%i" />' % (
                #         vehNr, i), file=routes)
                #     vehNr += 1
                #     lastVeh = i
            print("</routes>", file=routes)

    def generate_routefile_three_lanes(self):
        # np.random.seed(42)  # make tests reproducible
        N = 200  # number of time steps
        # demand per second from different directions
        pN1 = 1. / 4
        pN2 = 1. / 3
        pS1 = 1. / 3
        pS2 = 2. / 3
        with open("./three_lanes/three_lanes.rou.xml", "w") as routes:
            print("""<routes>
            <vType accel="1.0" decel="5.0" id="Car" length="4.0" maxSpeed="20.0" sigma="0.0" departSpeed="13.89"/>

            <route id="n2s" edges="gneE2 -gneE3" />
            <route id="s2n" edges="gneE3 -gneE2" />
            <route id="w2e" edges="gneE0 -gneE1" />
            <route id="w2n" edges="gneE0 -gneE2" />

            """, file=routes)
            lastVeh = 0
            vehNr = 0
            for i in range(N):
                if np.random.uniform(0, 1) < pN1:
                    print('    <vehicle id="right_%i" type="Car" route="n2s" depart="%i"  departLane="free"/>' % (vehNr, i), file=routes)
                    vehNr += 1
                    lastVeh = i
                if np.random.uniform(0, 1) < pN2:
                    print('    <vehicle id="right_%i" type="Car" route="n2s" depart="%i"  departLane="free"/>' % (vehNr, i), file=routes)
                    vehNr += 1
                    lastVeh = i
                if i == 5:
                    print('    <vehicle id="ego" type="Car" route="w2n" depart="%i" />' % i, file=routes)
                    vehNr += 1
                    lastVeh = i
                if np.random.uniform(0, 1) < pS1:
                    print('    <vehicle id="left_%i" type="Car" route="s2n" depart="%i"  departLane="free"/>' % (
                        vehNr, i), file=routes)
                    vehNr += 1
                    lastVeh = i
                # if np.random.uniform(0, 1) < pS2:
                #     print('    <vehicle id="left_%i" type="Car" route="s2n" depart="%i"  departLane="free"/>' % (
                #         vehNr, i), file=routes)
                #     vehNr += 1
                #     lastVeh = i
            print("</routes>", file=routes)





    def generate_routefile_two_intersections(self):
        # np.random.seed(42)  # make tests reproducible
        N = 200  # number of time steps
        # demand per second from different directions
        pN1 = 1. / 3
        pN2 = 1. / 3
        pS1 = 1. / 3
        pS2 = 1. / 3
        with open("./two_intersections/two_intersections.rou.xml", "w") as routes:
            print("""<routes>
            <vType accel="1.0" decel="5.0" id="Car" length="4.0" maxSpeed="20.0" sigma="0.0" departSpeed="13.89"/>

            <route id="n2s" edges="gneE19 -gneE18" />
            <route id="s2n" edges="gneE18 -gneE19" />
            <route id="w2e" edges="gneE16 -gneE32 -gneE31" />
            <route id="n2s2" edges="-gneE30 -gneE29" />
            <route id="s2n2" edges="gneE29 gneE30" />

            """, file=routes)
            lastVeh = 0
            vehNr = 0
            for i in range(N):
                if np.random.uniform(0, 1) < pN1:
                    print('    <vehicle id="right_%i" type="Car" route="n2s" depart="%i" />' % (vehNr, i), file=routes)
                    vehNr += 1
                    lastVeh = i
                if np.random.uniform(0, 1) < pN2:
                    print('    <vehicle id="right_%i" type="Car" route="s2n" depart="%i" />' % (vehNr, i), file=routes)
                    vehNr += 1
                    lastVeh = i

                if np.random.uniform(0, 1) < pS1:
                    print('    <vehicle id="left_%i" type="Car" route="n2s2" depart="%i" />' % (
                        vehNr, i), file=routes)
                    vehNr += 1
                    lastVeh = i
                if np.random.uniform(0, 1) < pS2:
                    print('    <vehicle id="left_%i" type="Car" route="s2n2" depart="%i" />' % (
                        vehNr, i), file=routes)
                    vehNr += 1
                    lastVeh = i
                if i == 5:
                    print('    <vehicle id="ego" type="Car" route="w2e" depart="%i" />' % i, file=routes)
                    vehNr += 1
                    lastVeh = i

            print("</routes>", file=routes)










