#!/bin/sh

rllib rollout ~/checkpoints/op3-001/PPO_RobotisOp3-v0_63bd7_00000_0_2021-05-22_19-13-44/checkpoint_000236/checkpoint-236 --run PPO --env RobotisOp3-v0 --episodes 500
