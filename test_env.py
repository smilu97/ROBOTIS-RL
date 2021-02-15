#!/usr/bin/env python

import numpy as np

from env import Environment

launchfile = '/home/smilu97/robotis/op3.launch'

def main():
    env = Environment(launchfile)
    epochs = 1000
    
    for num_epoch in xrange(epochs):
        env.reset()
        while True:
            action = 0.5 * (np.random.rand(20) - 0.5)
            state, reward, done, info = env.step(action)
            if done: break

if __name__ == '__main__':
    main()
