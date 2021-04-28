#!/usr/bin/env python

import numpy as np
import time
from linear_env import Op3LinearEnvironment
from logistic_env import Op3LogisticEnvrionment

launchfile = './op3.launch'

def main():
    # env = Op3LogisticEnvrionment(launchfile, speed=0.1)
    env = Op3LinearEnvironment(launchfile)
    epochs = 1000
    time.sleep(5)
    episode = 1
    for num_epoch in range(epochs):
        print('epsiode {} start'.format(episode))
        env.reset()
        i = 0
        T = 50
        while True:
            # action = np.zeros(env.action_size)
            # action[(i // T) % env.action_size] = 0.5 * np.sin(2 * np.pi * (i % T) / T) # all
            # action[4] = -0.5 + 0.5 * np.sin(2 * np.pi * i / T) # l_hip_pitch
            # action[7] =  0.5 + 0.5 * np.sin(2 * np.pi * i / T) # l_knee
            # action = 1.2 * (np.random.rand(env.action_size) - 0.5)
            action = (env.action_space.high - env.action_space.low) * np.random.rand(env.action_space.shape[0]) + env.action_space.low
            action = 2 * np.random.rand(env.action_size) - 1
            state, reward, done, info = env.step(action)
            time.sleep(1.0 / 30)
            i += 1
            # if done: break
        episode += 1

if __name__ == '__main__':
    main()
