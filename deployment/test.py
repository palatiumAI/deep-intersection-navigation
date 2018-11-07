
from __future__ import absolute_import
from __future__ import print_function

import logging
import os
import sys
import numpy as np
import operator
from collections import defaultdict
import json


from tensorforce.agents import PPOAgent
from tensorforce.agents import DQNAgent
from tensorforce.execution import Runner
from tensorforce.agents import agents as AgentsDictionary, Agent



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
import sumolib


from test_engine import Engine



if __name__ == "__main__":
    t_env = Engine('ego')

    #with open("./agent_specs/ppo.json", "r") as fp:
    with open("../agent_specs/ppo.json", "r") as fp:
            agent_config = json.load(fp=fp)


    #with open("./agent_specs/mlp_lstm_mlp.json", "r") as fp:
    with open("../agent_specs/mlp2_network.json", "r") as fp:
    #with open("./agent_specs/mlp2_lstm_network.json", "r") as fp:
            network = json.load(fp=fp)

    agent = Agent.from_spec(
        spec=agent_config,
        kwargs=dict(
            states=t_env.states,
            actions=t_env.actions,
            network=network
        )
    )

    agent.restore_model(directory="../models/aa_scarcity_nearest_2")

    collisionCounter = []
    disruptionCounter = []
    terminationTime = []
    co2 = []
    _ = []

    def episode_finished(r):

        #global collisionCounter
        #if t_env.getCollisionCounter():
        collisionCounter.append(t_env.getCollisionCounter())
        disruptionCounter.append(sum(t_env.getDisruptionIntensity()))
        terminationTime.append(t_env.getTerminationTime() / 1000)
        co2.append(t_env.getCO2())
        #print(sum(co2)/ len(co2))
        if len(collisionCounter) == num_episodes:
            print('Avg. Collisions: ',(sum(collisionCounter) / len(collisionCounter)) * 100, '%')
            print('Avg. braking: ',(-sum(disruptionCounter) / len(disruptionCounter)) / 100, 'm/s')
            print('Avg. time: ',sum(terminationTime) / len(terminationTime),'s')

            runner.close()

        print("Finished episode ", len(collisionCounter))
        #.format(ep=len(collisionCounter), reward=round(r.episode_rewards[-1],2)))
        return True

    traci.start(['sumo-gui', "-c", "../mapfiles/one_lane/one_lane.sumocfg","--start", '--no-step-log', 'true', '-W', 'true'])
    num_episodes = 1000
    runner = Runner(agent, t_env)
    runner.run(num_episodes=42, max_episode_timesteps=10000, episode_finished=episode_finished, deterministic=True, testing=True)


    ###
    #traci.start(['sumo-gui', "-c", "one_intersection_w_priority/one_intersection_w_priority.sumocfg", "--start"])
    # for i in range(20):
    #
    #     terminal = False
    #
    #     state = t_env.reset()
    #     while not terminal:
    #         action = agent.act(state)
    #         if action == 0:
    #             print('- dec')
    #         elif action == 1:
    #             print('+ acc')
    #         #elif action == 2:
    #          #   print('- eme -')
    #         else:
    #             print('= rem')
    #         state, terminal, reward = t_env.execute(action)
    #         #print(state)
    #         #print(reward)






















    # sumo_env = Engine_Test('ego')
    #
    #
    # sumo_agent = DQNAgent(
    #     states=sumo_env.states,
    #     actions=sumo_env.actions,
    #     network=[
    #         dict(type='flatten'),
    #         dict(type='dense', size=32),
    #         dict(type='dense', size=32)
    #     ],
    #     batching_capacity=1000,
    #     actions_exploration={
    #     "type": "epsilon_anneal",
    #     "initial_epsilon": 0.5,
    #     "final_epsilon": 0.0,
    #     "timesteps": 10000
    # },
    # )
    #
    #
    # runner = Runner(sumo_agent, sumo_env)
    #
    #
    #
    #
    # traci.start(['sumo-gui', "-c", "huge_crossing/huge_crossing.sumocfg"])
    #
    # runner.run(episodes=1, max_episode_timesteps=200, episode_finished=episode_finished)
    # runner.close()





















