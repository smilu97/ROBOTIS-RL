#!/usr/bin/env python

import numpy as np
import time
from env import Environment

launchfile = '/home/smilu97/robotis/op3.launch'

def main():
    env = Environment(launchfile)
    epochs = 1000
    time.sleep(5)
    episode = 1
    for num_epoch in xrange(epochs):
        print 'epsiode {} start'.format(episode)
        env.reset()
        time.sleep(0.3)
        while True:
            action = 1.0 * (np.random.rand(env.action_size) - 0.5)
            state, reward, done, info = env.step(action)
            if done: break
        episode += 1

if __name__ == '__main__':
    main()
