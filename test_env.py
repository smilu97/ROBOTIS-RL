#!/usr/bin/env python

import numpy as np
import time
from env import Environment

launchfile = '/home/smilu97/robotis/op3.launch'

def main():
    env = Environment(launchfile)
    epochs = 1000
    time.sleep(5)
    action = 0.5 * (np.random.rand(19) - 0.5)
    for num_epoch in xrange(epochs):
        env.reset()
        time.sleep(3.0)
        while True:
            state, reward, done, info = env.step(action)
            if done: break
            time.sleep(1.0)

if __name__ == '__main__':
    main()
