#!/usr/bin/env python

import numpy as np
from ros import RosController

def main():
    r = RosController('/home/smilu97/robotis/op3.launch')
    r.wait_for_controllers()
    while True:
        r.reset()
        for _ in xrange(1000):
            action = 0.2 * (np.random.rand(20) - 0.5)
            r.unpause()
            r.publish_action(action)
            data = r.get_link_states()
            print('data:', data)
            r.pause()

if __name__ == '__main__':
    main()
