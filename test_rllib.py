#!/usr/bin/env python

import ray
from ray import tune
from ray.rllib.agents.ppo import PPOTrainer
from ray.tune.registry import register_env
from rllib_config import config
from env import OP3Env
import gym
import pybulletgym

def env_creator(env_config):
    return OP3Env()

register_env("RobotisOp3-v0", env_creator)
# config["env"] = "RobotisOp3-v0"
config["num_workers"] = 1

ray.init()
agent = PPOTrainer(config=config, env=OP3Env)
agent.restore('/home/smilu97/checkpoints/op3-001/PPO_RobotisOp3-v0_63bd7_00000_0_2021-05-22_19-13-44/checkpoint_000236/checkpoint-236')

# instantiate env class
env = OP3Env()

while True:
    # run until episode ends
    episode_reward = 0
    done = False
    env.render(mode='human')
    obs = env.reset()
    while not done:
        action = agent.compute_action(obs)
        obs, reward, done, info = env.step(action)
        episode_reward += reward
