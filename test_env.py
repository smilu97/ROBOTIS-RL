#!/usr/bin/env python

import numpy as np
import time
from matplotlib import pyplot as plt
from env import OP3Env

def main():
    env = OP3Env()
    epochs = 1000
    episode = 1
    sl = env.sl
    op3c = env.op3c
    
    env.reset()

    for num_epoch in range(epochs):
        print('epsiode {} start'.format(episode))
        i = 0
        T = 50
        t_joint = np.random.randint(18)
        pos = []
        vel = []
        eff = []
        act = []
        done = False
        while not done:
            action = (env.action_space.high - env.action_space.low) * np.random.rand(env.action_space.shape[0]) + env.action_space.low
            state, reward, done, info = env.step(action)

            pos.append(state[t_joint])
            vel.append(state[t_joint+ 1 * sl])
            eff.append(state[t_joint+ 2 * sl])
            act.append(action[t_joint])
            
            i += 1

        env.reset()
        if False:
            line_pos, = plt.plot(pos, label='pos')
            line_vel, = plt.plot(vel, label='vel')
            line_eff, = plt.plot(eff, label='eff')
            line_act, = plt.plot(act, label='act')
            plt.legend(hanop3c.op3_module_names[t_joint])
            plt.show()
        episode += 1

if __name__ == '__main__':
    main()
