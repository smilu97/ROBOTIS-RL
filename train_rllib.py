#!/usr/bin/env python

from ray import tune
from ray.rllib.agents.ppo import PPOTrainer
from ray.rllib.agents.sac import SACTrainer
from ray.tune.registry import register_env
from ppo_config import config as ppo_config
from sac_config import config as sac_config
from env import OP3Env
import gym
import pybulletgym

def env_creator(env_config):
    return OP3Env(use_bias=True, human_bias=True)

trainer = 'ppo'

config = ppo_config if trainer == 'ppo' else sac_config
t = PPOTrainer if trainer == 'ppo' else SACTrainer

register_env("RobotisOp3-v0", env_creator)
config["env"] = "RobotisOp3-v0"

tune.run(
    t,
    name='timeloop3',
    resume=True,
    config=config,
    checkpoint_freq=20,
    checkpoint_at_end=True)
    
