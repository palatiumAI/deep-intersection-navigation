
from __future__ import absolute_import
from __future__ import print_function

import logging
import os
import sys
import numpy as np
import operator
from collections import defaultdict
import json
import time



from tensorforce.agents import PPOAgent
#from tensorforce.agents import DQNAgent
#from tensorforce.execution import Runner
#from tensorforce.execution import ThreadedRunner
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


from engine import Engine




if __name__ == "__main__":


    env = Engine('ego')

    num_episodes = 5



    with open("../agent_specs/ppo.json", "r") as fp:
            agent_config = json.load(fp=fp)

    #with open("./agent_specs/mlp2_lstm_network.json", "r") as fp:
    with open("../agent_specs/mlp2_network.json", "r") as fp:

    #with open("./agent_specs/mlp_lstm_mlp.json", "r") as fp:
            network = json.load(fp=fp)



    agent = Agent.from_spec(
        spec=agent_config,
        kwargs=dict(
            states=env.states,
            actions=env.actions,
            network=network,

    )
    )


    runner = Runner(agent, env)


    _ = [0]
    start_time = time.time()
    def episode_finished(r):
        print("Finished episode {ep}/{max} ({perc}%) after {ts} timesteps (reward: {reward}) | Duration: {m}s | Average reward (500): {avg}".format(ep=r.episode, ts=r.episode_timestep, max=num_episodes, perc=np.round((r.episode/num_episodes)*100,1),
                                                                               reward=round(r.episode_rewards[-1],2), m=round(time.time() - start_time - _[-1],2), avg=np.round(np.mean(runner.episode_rewards[-500:]),1)))
        _.append(time.time() - start_time)
        #if r.episode == num_episodes:
         #   r.agent.save_model(directory="./models/")
        #env.setDivisor(np.minimum(round(r.episode * 20), num_episodes * 0.1) / num_episodes)

        # 1,000,000 * 0.2 = 200,000 ; 200,000 / 1,000,000 = 1/5
        # 200,000 / 8 = 25,000 -> * 8

        env.setDivisor(1/10)
        return True


    print("Starting {agent} for Environment '{env}'".format(agent=agent, env=env))

    traci.start(['sumo', "-c", "../mapfiles/one_lane/one_lane.sumocfg", '--no-step-log', 'true', "--start", '-W', 'true', '--time-to-teleport', '60' ])



    runner.run(episodes=num_episodes, max_episode_timesteps=10000, episode_finished=episode_finished)

    print("Learning finished. Total episodes: {ep}. Average reward of last 1000 episodes: {ar}.".format(
        ep=runner.episode,
        ar=np.mean(runner.episode_rewards[-1000:])))
    runner.close()








