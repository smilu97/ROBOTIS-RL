#!/usr/bin/env python

from ray import tune
from ray.rllib.agents.ppo import PPOTrainer
from ray.tune.registry import register_env
from rllib_config import config

import env

config["env"] = "RobotisOp3-v0"

tune.run(
    PPOTrainer,
    resume=False,
    local_dir='~/.ckp/op3-050409',
    config=config,
    checkpoint_freq=10,
    checkpoint_at_end=True)
