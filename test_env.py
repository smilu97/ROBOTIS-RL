#!/usr/bin/env python

import numpy as np
import time
from linear_env import Op3LinearEnvironment
from logistic_env import Op3LogisticEnvrionment

launchfile = '/home/smilu97/robotis/op3.launch'

def main():
    env = Op3LogisticEnvrionment(launchfile, speed=0.1)
    epochs = 1000
    time.sleep(5)
    episode = 1
    for num_epoch in range(epochs):
        print('epsiode {} start'.format(episode))
        env.reset()
        while True:
            action = 1.0 * (np.random.rand(env.action_size) - 0.5)
            state, reward, done, info = env.step(action)
            time.sleep(1.0 / 30)
            if done: break
        episode += 1

if __name__ == '__main__':
    main()
