#!/usr/bin/env python

import numpy as np
import gym

def main():
    env = gym.envs.make('Humanoid-v2')
    while True:
        env.reset()
        while True:
            action = env.action_space.sample()
            env.render()
            state, reward, done, info = env.step(action)
            if done: break

if __name__ == '__main__':
    main()
