from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

from action import Action
from Deep_Intersection_Navigation.representation.assembler import Assembler
from reward import Reward
import sumolib
import traci
import numpy as np
#from helper import Helper


from tensorforce.environments import Environment



class Engine(Environment):

    def __init__(self, carID):
        self.carID = carID
        self.output = None
        self.collisionCounter = False
        self.disruptionIntensity = []
        self.terminationTime = []
        self.co2 = []

    def __str__(self):
        return 'Test Engine'


    def reset(self):

        self.generate_routefile()
        traci.load(["-c", "../mapfiles/one_lane/one_lane.sumocfg", "--collision.check-junctions", "1", "--start", '--no-step-log', 'true', '-W', 'true'])




        #self.generate_routefile_three_lanes()
        #traci.load(["-c", "three_lanes/three_lanes.sumocfg", "--collision.check-junctions", "1","--start", '--no-step-log', 'true', '-W', 'true'])

        #self.generate_routefile_three_lanes_left()
        #traci.load(["-c", "three_lanes/three_lanes2.sumocfg", "--collision.check-junctions", "1", "--start", '--no-step-log', 'true', '-W', 'true'])


        #self.generate_routefile_three_lanes()
        #traci.load(["-c", "three_lanes/three_lanes2.sumocfg", "--collision.check-junctions", "1", "--start", '--no-step-log', 'true', '-W', 'true'])


        #self.generate_routefile_two_intersections()
        #traci.load(["-c", "two_intersections/two_intersections.sumocfg", "--collision.check-junctions", "1","--start", '--no-step-log', 'true', '-W', 'true'])


        ###
        #traci.load(["-c", "one_intersection_w_priority/one_intersection_w_priority.sumocfg", "--collision.check-junctions", "1", "--start"])

        run = Assembler(self.carID)

        self.collisionCounter = False

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

    def setCollisionCounter(self):
        self.collisionCounter = True


    def getCollisionCounter(self):
        return self.collisionCounter

    def getDisruptionIntensity(self):
        return self.disruptionIntensity

    def getTerminationTime(self):
        return self.terminationTime[-1]

    def getCO2(self):
        return sum(self.co2) #/ len(self.co2)

    def execute(self, action):
        run = Assembler(self.carID)
        self.output = run.getTensor()
        #print(run.getTensor())

        if self.output is None:
            term = True
            return self.getObservation(), term, 0
        else:
            self.co2.append(traci.vehicle.getFuelConsumption(self.carID))

        rew = Reward('ego', run.getTraffic(), self.output, run.getPrioritizedTraffic())

        coll, term = rew.collision_test()
        if term is True:
            self.setCollisionCounter()
            cost = coll
            return self.getObservation(), term, cost

        traci.vehicle.setSpeedMode('ego', 0)
        num_vec = traci.vehicle.getIDList()
        disruption = []
        for i in range(len(num_vec)):
            if num_vec[i] != 'ego':
                if traci.vehicle.getAcceleration(num_vec[i]) < 0:
                    disruption.append(traci.vehicle.getAcceleration(num_vec[i]))

                traci.vehicle.setLaneChangeMode(num_vec[i], 512)

        self.disruptionIntensity.append(sum(disruption))
            ###
            #if i % 2 == 0:
                #traci.vehicle.setSpeedMode(num_vec[i], 23)


        carID = 'ego'
        act = Action(carID)
        if action == 0:
            act.decelerate()
            #print('dec')
        elif action == 1:
            act.accelerate()
            #print('acc')
        #elif action == 2:
         #   print('eme')
          #  act.emergency()
        else:
            act.remain()
            #print('rem')
        #print(traci.vehicle.getSpeed(self.carID))
        #gap, term = rew.emergency_gap_test()
        #wary = rew.wary_before_intersection()


        #win = rew.target()
        #brake = rew.emergency_brake()
        cost = rew.optimum_speed_deviation() #+ gap #+ brake # #+ wary
        traci.simulationStep()
        #print(self.output)
        self.terminationTime.append(traci.simulation.getCurrentTime())
        #print(traci.simulation.getCurrentTime())

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
        pN1 = 1. / 10
        pN2 = 1. / 3
        pS1 = 1. / 10
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
        pN1 = 1. / 8
        pN2 = 1. / 8
        pS1 = 1. / 8
        pS2 = 1. / 8
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
                    print('    <vehicle id="right_%i" type="Car" route="n2s" depart="%i"  departLane="free" departSpeed="13.89"/>' % (vehNr, i), file=routes)
                    vehNr += 1
                    lastVeh = i
                if np.random.uniform(0, 1) < pN2:
                    print('    <vehicle id="right_%i" type="Car" route="n2s" depart="%i"  departLane="free" departSpeed="13.89"/>' % (vehNr, i), file=routes)
                    vehNr += 1
                    lastVeh = i
                if i == 15:
                    print('    <vehicle id="ego" type="Car" route="w2e" depart="%i" departSpeed="13.89"/>' % i, file=routes)
                    vehNr += 1
                    lastVeh = i
                if np.random.uniform(0, 1) < pS1:
                    print('    <vehicle id="left_%i" type="Car" route="s2n" depart="%i"  departLane="free" departSpeed="13.89"/>' % (
                        vehNr, i), file=routes)
                    vehNr += 1
                    lastVeh = i
                if np.random.uniform(0, 1) < pS2:
                    print('    <vehicle id="left_%i" type="Car" route="s2n" depart="%i"  departLane="free" departSpeed="13.89"/>' % (
                        vehNr, i), file=routes)
                    vehNr += 1
                    lastVeh = i
            print("</routes>", file=routes)

    def generate_routefile_three_lanes_left(self):
        # np.random.seed(42)  # make tests reproducible
        N = 200  # number of time steps
        # demand per second from different directions
        pN1 = 1. / 8
        pN2 = 1. / 8
        pS1 = 1. / 8
        pS2 = 1. / 8
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
                    print('    <vehicle id="right_%i" type="Car" route="n2s" depart="%i"  departLane="free" departSpeed="13.89"/>' % (vehNr, i), file=routes)
                    vehNr += 1
                    lastVeh = i
                if np.random.uniform(0, 1) < pN2:
                    print('    <vehicle id="right_%i" type="Car" route="n2s" depart="%i"  departLane="free" departSpeed="13.89"/>' % (vehNr, i), file=routes)
                    vehNr += 1
                    lastVeh = i
                if i == 15:
                    print('    <vehicle id="ego" type="Car" route="w2n" depart="%i" departSpeed="13.89"/>' % i, file=routes)
                    vehNr += 1
                    lastVeh = i
                if np.random.uniform(0, 1) < pS1:
                    print('    <vehicle id="left_%i" type="Car" route="s2n" depart="%i"  departLane="free" departSpeed="13.89"/>' % (
                        vehNr, i), file=routes)
                    vehNr += 1
                    lastVeh = i
                if np.random.uniform(0, 1) < pS2:
                    print('    <vehicle id="left_%i" type="Car" route="s2n" depart="%i"  departLane="free" departSpeed="13.89"/>' % (
                        vehNr, i), file=routes)
                    vehNr += 1
                    lastVeh = i
            print("</routes>", file=routes)




    def generate_routefile_two_intersections(self):
        # np.random.seed(42)  # make tests reproducible
        N = 200  # number of time steps
        # demand per second from different directions
        pN1 = 1. / 6
        pN2 = 1. / 5
        pS1 = 1. / 5
        pS2 = 1. / 6
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
                if i == 15:
                    print('    <vehicle id="ego" type="Car" route="w2e" depart="%i" departSpeed="13.89"/>' % i, file=routes)
                    vehNr += 1
                    lastVeh = i

            print("</routes>", file=routes)














