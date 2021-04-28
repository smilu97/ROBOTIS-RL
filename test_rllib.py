#!/usr/bin/env python

from ray import tune
from ray.rllib.agents.ppo import PPOTrainer
from ray.tune.registry import register_env
from linear_env import Op3LinearEnvironment
from rllib_config import config

launchfile = './op3.launch'

def env_creator(env_config):
    return Op3LinearEnvironment(launchfile)

register_env("RobotisOp3-v0", env_creator)

config["env"] = "RobotisOp3-v0"

tune.run(PPOTrainer, config=config)
