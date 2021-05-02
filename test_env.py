#!/usr/bin/env python

import numpy as np
import time
import op3constant as op3c
from matplotlib import pyplot as plt
from linear_env import Op3LinearEnvironment
from logistic_env import Op3LogisticEnvrionment

launchfile = './op3.launch'

def main():
    # env = Op3LogisticEnvrionment(launchfile, speed=0.1)
    env = Op3LinearEnvironment(launchfile)
    epochs = 1000
    time.sleep(5)
    episode = 1
    sl = len(op3c.op3_module_names)
    
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
        while True:
            action = (env.action_space.high - env.action_space.low) * np.random.rand(env.action_space.shape[0]) + env.action_space.low
            state, reward, done, info = env.step(action)

            pos.append(state[t_joint])
            vel.append(state[t_joint+ 1 * sl])
            eff.append(state[t_joint+ 2 * sl])
            act.append(action[t_joint])

            time.sleep(1.0 / 30)
            i += 1
            if done: break

        env.reset()
        if False:
            line_pos, = plt.plot(pos, label='pos')
            line_vel, = plt.plot(vel, label='vel')
            line_eff, = plt.plot(eff, label='eff')
            line_act, = plt.plot(act, label='act')
            plt.legend(handles=[line_pos, line_act, line_eff, line_vel])
            plt.title(op3c.op3_module_names[t_joint])
            plt.show()
        episode += 1

if __name__ == '__main__':
    main()
