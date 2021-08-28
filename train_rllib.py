#!/usr/bin/env python

from ray import tune
from ray.rllib.agents.ppo import PPOTrainer
from ray.rllib.agents.sac import SACTrainer
from ray.rllib.agents.ars import ARSTrainer
from ray.tune.registry import register_env
from ppo_config import config as ppo_config
from sac_config import config as sac_config
from ars_config import config as ars_config
from env import OP3Env
import gym
import pybulletgym

def env_creator(env_config):
    return OP3Env(use_bias=False, human_bias=True)

trainer = 'sac'

config = {
    'ppo': ppo_config,
    'sac': sac_config,
    'ars': ars_config,
}.get(trainer, ppo_config)

trainer = {
    'ppo': PPOTrainer,
    'sac': SACTrainer,
    'ars': ARSTrainer,
}.get(trainer, PPOTrainer)

register_env("RobotisOp3-v0", env_creator)
config["env"] = "RobotisOp3-v0"

tune.run(
    t,
    name='sac_bias',
    resume=False,
    config=config,
    checkpoint_freq=20,
    checkpoint_at_end=True)
    
