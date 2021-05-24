#!/usr/bin/env python

import ray
from ray import tune
from ray.rllib.agents.ppo import PPOTrainer
from ray.tune.registry import register_env
from ppo_config import config
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
agent.restore('~/checkpoints/op3-002/PPO_RobotisOp3-v0_b1556_00000_0_2021-05-24_00-58-19/checkpoint_000581/checkpoint-581')

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
