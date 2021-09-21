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
    return OP3Env(use_bias=True, human_bias=False)

trainer_name = 'sac'

config = {
    'ppo': ppo_config,
    'sac': sac_config,
    'ars': ars_config,
}.get(trainer_name, ppo_config)

trainer = {
    'ppo': PPOTrainer,
    'sac': SACTrainer,
    'ars': ARSTrainer,
}.get(trainer_name, PPOTrainer)

register_env("RobotisOp3-v0", env_creator)
config["env"] = "RobotisOp3-v0"

tune.run(
    trainer,
    name='saclazy2',
    restore='/home/smilu97/ray_results/saclazy2/SAC_RobotisOp3-v0_ca72d_00000_0_2021-09-15_03-15-50/checkpoint_027840/checkpoint-27840',
    config=config,
    checkpoint_freq=20,
    checkpoint_at_end=True)
    
