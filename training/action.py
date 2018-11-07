

import sys
import os

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



class Action:

    #actions = 3

    def __init__(self, carID):
        self.carID = carID

    # def accelerate(self):
    #     current_speed = traci.vehicle.getSpeed(self.carID)
    #     target_speed = current_speed + 2
    #     traci.vehicle.setSpeed(self.carID, target_speed)

    def accelerate(self):
        current_speed = traci.vehicle.getSpeed(self.carID)
        if current_speed >= 17:
            target_speed = current_speed + 0.3
            traci.vehicle.setSpeed(self.carID, target_speed)
        else:
            target_speed = current_speed + 2
            traci.vehicle.setSpeed(self.carID, target_speed)

    def decelerate(self):
        current_speed = traci.vehicle.getSpeed(self.carID)
        if current_speed > 1.5:
            target_speed = current_speed - 2
            traci.vehicle.setSpeed(self.carID, target_speed)
        else:
            traci.vehicle.setSpeed(self.carID, 0)

    def emergency(self):
        current_speed = traci.vehicle.getSpeed(self.carID)
        target_speed = current_speed - 3.5
        traci.vehicle.setSpeed(self.carID, target_speed)

    def remain(self):
        current_speed = traci.vehicle.getSpeed(self.carID)
        target_speed = current_speed
        traci.vehicle.setSpeed(self.carID, target_speed)

