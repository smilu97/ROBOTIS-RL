#!/usr/bin/env python

import numpy as np
import time
from env import Environment

launchfile = '/home/smilu97/robotis/op3.launch'

def main():
    env = Environment(launchfile)
    epochs = 1000
    
    for num_epoch in xrange(epochs):
        env.reset()
        action = [np.random.randint(0,3) for _ in range(20)]
        while True:
            # time.sleep(1.0 / 50)
            state, reward, done, info = env.step(action)
            if done: break

if __name__ == '__main__':
    main()
