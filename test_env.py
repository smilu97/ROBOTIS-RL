#!/usr/bin/env python

import numpy as np
import time
import gym
import env
import pybulletgym
import argparse
from matplotlib import pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument('--random-action', '-r', action='store_true')
parser.add_argument('--fixed-port', '-f', action='store_true')
parser.add_argument('--blind', '-b', action='store_true')

def main():
    args = parser.parse_args()
    random_action = args.random_action
    fixed_port = args.fixed_port

    env = gym.make('RobotisOp3-v0',
        random_port=not fixed_port,
        print_rewards=True,
        use_bias=True,
    )
    epochs = 1000
    episode = 1
    sl = env.action_space.shape[0]
    
    if not args.blind:
        env.render(mode='human')

    for num_epoch in range(epochs):
        print('epsiode {} start'.format(episode))
        obs = env.reset()
        done = False
        acc_reward = 0.0
        num_steps = 0
        while not done:
            if random_action:
                action = 2 * np.random.rand(sl) - 1
            else:
                action = np.zeros(sl)
            obs, reward, done, info = env.step(action)
            acc_reward += reward
            num_steps += 1
        
        print('reward:', acc_reward)
        print('num steps:', num_steps)
        episode += 1

if __name__ == '__main__':
    main()
